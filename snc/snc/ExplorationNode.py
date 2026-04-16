#!/usr/bin/env python3

import math
import collections
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty, String
from visualization_msgs.msg import MarkerArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from snc_interfaces.srv import ExplorationControl
from snc.constants import (
    TRIGGER_HOME_BUFFER_SIZE,
    TRIGGER_HOME_INTERFACE,
    TRIGGER_HOME_TOPIC,
    RETURN_HOME_TRAJECTORY_BUFFER_SIZE, 
    RETURN_HOME_TRAJECTORY_TOPIC,
    RETURN_HOME_TRAJECTORY_INTERFACE,
    EXPLORE_BREADCRUMBS_TOPIC,
    EXPLORE_BREADCRUMBS_INTERFACE,
    EXPLORE_BREADCRUMBS_BUFFER_SIZE,
    RETURN_BREADCRUMBS_TOPIC,
    RETURN_BREADCRUMBS_INTERFACE,
    RETURN_BREADCRUMBS_BUFFER_SIZE
)

MAP_UNKNOWN = -1
MAP_FREE = 0
MAP_OCCUPIED_THRESHOLD = 50

STATE_IDLE = 'STATUS_IDLE'
STATE_EXPLORING = 'STATUS_EXPLORING'
STATE_SPINNING = 'STATUS_SPINNING'
STATE_TELEOP = 'STATUS_TELEOP'
STATE_RETURNING = 'STATUS_RETURNING'
STATE_DONE = 'STATUS_DONE'


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.navigator = BasicNavigator()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.global_frame = 'map'
        self.robot_base_frame = 'base_link'

        self.declare_parameter('planner_frequency', 1.0)
        self.declare_parameter('status_frequency', 1.0)
        self.declare_parameter('exploration_timeout_sec', 2000.0)
        self.declare_parameter('spin_angular_speed', 0.8)
        self.declare_parameter('spin_angle_deg', 360.0)
        self.declare_parameter('min_frontier_cluster_size', 50)
        self.declare_parameter('frontier_standoff_m', 0.1)

        self.planner_frequency = float(self.get_parameter('planner_frequency').value)
        self.status_frequency = float(self.get_parameter('status_frequency').value)
        self.exploration_timeout_sec = float(self.get_parameter('exploration_timeout_sec').value)
        self.spin_angular_speed = float(self.get_parameter('spin_angular_speed').value)
        self.spin_angle_deg = float(self.get_parameter('spin_angle_deg').value)
        self.min_frontier_cluster_size = int(self.get_parameter('min_frontier_cluster_size').value)
        self.frontier_standoff_m = float(self.get_parameter('frontier_standoff_m').value)

        self.latest_map = None
        self.goal_active = False
        self.is_ready = False

        self.state = STATE_IDLE
        self.started = False
        self.exploration_start_time = None
        self.hazard_ids = set()
        self.pending_resume_after_spin = False

        self.spin_end_time = None
        self.no_frontier_count = 0
        self.no_frontier_limit = 5

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.start_sub = self.create_subscription(
            Empty,
            '/snc_start',
            self.start_callback,
            10
        )

        self.hazard_signal_sub = self.create_subscription(
            Empty,
            '/snc_hazard_signal',
            self.hazard_signal_callback,
            10
        )

        self.teleop_sub = self.create_subscription(
            Empty,
            '/trigger_teleop',
            self.teleop_callback,
            10
        )

        self.hazards_sub = self.create_subscription(
            MarkerArray,   # replace if your /hazards topic uses another type
            '/hazards',
            self.hazards_callback,
            10
        )

        self.path_tracing_sub = self.create_subscription(
            EXPLORE_BREADCRUMBS_INTERFACE,
            EXPLORE_BREADCRUMBS_TOPIC,
            self.path_explore_callback,
            10
        )

        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.return_pub = self.create_publisher(Empty, '/snc_return_trigger', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.control_srv = self.create_service(
            ExplorationControl,
            '/snc_exploration_control',
            self.control_service_callback
        )

        self.plan_timer = self.create_timer(
            1.0 / self.planner_frequency,
            self.plan_step
        )

        self.status_timer = self.create_timer(
            1.0 / self.status_frequency,
            self.publish_status
        )

        self.get_logger().info('Exploration node created')

    # ---------- readiness ----------
    def wait_until_ready(self, executor: MultiThreadedExecutor):
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active(localizer='slam_toolbox')
        self.get_logger().info('Nav2 is active')

        self.get_logger().info(
            f'Waiting for TF {self.global_frame} -> {self.robot_base_frame}...'
        )
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if self.tf_buffer.can_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.1)
            ):
                break

        self.get_logger().info('Robot pose TF is available')

        self.get_logger().info('Waiting for occupancy grid /map...')
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if (
                self.latest_map is not None
                and self.latest_map.info.width > 0
                and self.latest_map.info.height > 0
                and len(self.latest_map.data) ==
                   self.latest_map.info.width * self.latest_map.info.height
            ):
                break

        self.is_ready = True
        self.get_logger().info('Exploration node is ready and waiting for /snc_start')

    # ---------- callbacks ----------
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def start_callback(self, _msg: Empty):
        if not self.is_ready:
            return
        if self.state in [STATE_EXPLORING, STATE_SPINNING]:
            return
        self.start_exploration()

    def hazard_signal_callback(self, _msg: Empty):
        if self.state != STATE_EXPLORING:
            return

        self.get_logger().info('Hazard signal received: cancel current goal and spin')
        self.cancel_navigation()
        self.start_spin()

    def teleop_callback(self, _msg: Empty):
        self.get_logger().info('Teleop trigger received')
        self.cancel_navigation()
        self.state = STATE_TELEOP
        self.pending_resume_after_spin = False

    def hazards_callback(self, msg: MarkerArray):
        # Example for MarkerArray: use marker.id as unique hazard ID.
        # Replace this logic if /hazards has a different message type.
        # TODO: Add mapping logic for the markers.
        before = len(self.hazard_ids)
        for marker in msg.markers:
            self.hazard_ids.add(int(marker.id))

        after = len(self.hazard_ids)
        if after != before:
            self.get_logger().info(f'Unique hazards confirmed: {after}')

        if after >= 5 and self.state not in [STATE_RETURNING, STATE_DONE]:
            self.trigger_return_home('5 unique markers confirmed')

    def control_service_callback(self, request, response):
        cmd = request.command.strip().upper()

        if cmd == 'STATUS':
            response.success = True
            response.state = self.state
            response.hazards_found = len(self.hazard_ids)
            response.message = 'Status returned'
            return response

        if not self.is_ready:
            response.success = False
            response.state = self.state
            response.hazards_found = len(self.hazard_ids)
            response.message = 'Node is not ready yet'
            return response

        if cmd == 'START':
            self.start_exploration()
            response.success = True
            response.state = self.state
            response.hazards_found = len(self.hazard_ids)
            response.message = 'Exploration started'
            return response

        if cmd == 'STOP':
            self.cancel_navigation()
            self.state = STATE_IDLE
            response.success = True
            response.state = self.state
            response.hazards_found = len(self.hazard_ids)
            response.message = 'Exploration stopped'
            return response

        if cmd == 'RESUME':
            if self.state in [STATE_IDLE, STATE_TELEOP]:
                self.state = STATE_EXPLORING
                response.success = True
                response.state = self.state
                response.hazards_found = len(self.hazard_ids)
                response.message = 'Exploration resumed'
                return response

        if cmd == 'TELEOP':
            self.cancel_navigation()
            self.state = STATE_TELEOP
            response.success = True
            response.state = self.state
            response.hazards_found = len(self.hazard_ids)
            response.message = 'Switched to teleop state'
            return response

        response.success = False
        response.state = self.state
        response.hazards_found = len(self.hazard_ids)
        response.message = f'Unsupported command: {request.command}'
        return response

    def path_explore_callback(self, msg):
        poses = msg.poses
        

    # ---------- exploration control ----------
    def start_exploration(self):
        self.started = True
        self.goal_active = False
        self.no_frontier_count = 0
        self.pending_resume_after_spin = False

        if self.exploration_start_time is None:
            self.exploration_start_time = self.get_clock().now()

        self.state = STATE_EXPLORING
        self.get_logger().info('Exploration started....')

    def trigger_return_home(self, reason: str):
        self.get_logger().info(f'Triggering return-home: {reason}')
        self.cancel_navigation()
        self.state = STATE_RETURNING
        self.return_pub.publish(Empty())

    def cancel_navigation(self):
        self.goal_active = False
        try:
            self.navigator.cancelTask()
        except Exception:
            pass

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    # ---------- spin behaviour ----------
    def start_spin(self):
        self.state = STATE_SPINNING
        self.pending_resume_after_spin = True

        total_angle_rad = math.radians(self.spin_angle_deg)
        duration_sec = total_angle_rad / max(0.01, abs(self.spin_angular_speed))
        self.spin_end_time = self.get_clock().now() + Duration(seconds=duration_sec)

    def update_spin(self):
        if self.state != STATE_SPINNING:
            return

        now = self.get_clock().now()
        twist = Twist()

        if now < self.spin_end_time:
            twist.angular.z = self.spin_angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        if self.pending_resume_after_spin:
            self.pending_resume_after_spin = False
            self.state = STATE_EXPLORING
            self.get_logger().info('Spin complete, resuming exploration')

    # ---------- robot pose ----------
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not get robot pose: {ex}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    # ---------- planning loop ----------
    def plan_step(self):
        if not self.is_ready:
            return

        self.update_spin()

        if self.state != STATE_EXPLORING:
            return

        if self.exploration_start_time is not None:
            elapsed = (self.get_clock().now() - self.exploration_start_time).nanoseconds / 1e9
            if elapsed >= self.exploration_timeout_sec:
                self.trigger_return_home('4-minute timeout reached')
                return

        if len(self.hazard_ids) >= 5:
            self.trigger_return_home('5 hazards found')
            return

        if self.goal_active:
            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            self.goal_active = False

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Frontier reached, replanning...')
            else:
                self.get_logger().warn(f'Navigation result: {result}, replanning...')

        curr_pose = self.get_robot_pose()
        if curr_pose is None or self.latest_map is None:
            return

        goal_pose = self.find_frontier_goal(self.latest_map, curr_pose)

        if goal_pose is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                f'No frontier found ({self.no_frontier_count}/{self.no_frontier_limit})'
            )
            if self.no_frontier_count >= self.no_frontier_limit:
                self.trigger_return_home('No frontiers left')
            return

        self.no_frontier_count = 0
        self.navigator.goToPose(goal_pose)
        self.goal_active = True

    # ---------- frontier extraction ----------
    def is_cell_traversable(self, value):
        return 0 <= int(value) < MAP_OCCUPIED_THRESHOLD

    def cell_has_unknown_neighbours(self, grid, x, y, width, height):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if int(grid[ny, nx]) == MAP_UNKNOWN:
                        return True
        return False

    def get_reachable_cells(self, grid, width, height, start_x, start_y):
        if not (0 <= start_x < width and 0 <= start_y < height):
            return set()

        if not self.is_cell_traversable(grid[start_y, start_x]):
            return set()

        reachable_cells = set()
        q = collections.deque([(start_x, start_y)])
        reachable_cells.add((start_x, start_y))

        while q:
            cx, cy = q.popleft()

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in reachable_cells:
                    continue
                if not self.is_cell_traversable(grid[ny, nx]):
                    continue
                
                reachable_cells.add((nx, ny))
                q.append((nx, ny))

        return reachable_cells

    def extract_frontier_cells(self, grid, width, height, reachable_cells):
        frontier_cells = []
        for x, y in reachable_cells:
            if self.cell_has_unknown_neighbours(grid, x, y, width, height):
                frontier_cells.append((x, y))

        return frontier_cells

    def rank_frontier_clusters(self, clusters, robot_x, robot_y):
        ranked = []

        for cluster in clusters:
            if len(cluster) < self.min_frontier_cluster_size:
                continue

            centroid_x = sum(c[0] for c in cluster) / len(cluster)
            centroid_y = sum(c[1] for c in cluster) / len(cluster)

            dist = math.hypot(centroid_x - robot_x, centroid_y - robot_y)
            size_bonus = 0.2 * len(cluster)
            score = dist - size_bonus

            ranked.append((score, cluster))

        ranked.sort(key=lambda x: x[0])
        return [cluster for _, cluster in ranked]

    def cluster_frontiers(self, frontier_cells):
        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            q = collections.deque([cell])
            visited.add(cell)

            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))

                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        n = (cx + dx, cy + dy)
                        if n in frontier_set and n not in visited:
                            visited.add(n)
                            q.append(n)

            clusters.append(cluster)

        return clusters

    def backoff_goal_cell(self, cluster, robot_x, robot_y):
        centroid_x = sum(c[0] for c in cluster) / len(cluster)
        centroid_y = sum(c[1] for c in cluster) / len(cluster)

        closest = min(
            cluster,
            key=lambda c: math.hypot(c[0] - centroid_x, c[1] - centroid_y)
        )

        fx, fy = closest
        vx = robot_x - fx
        vy = robot_y - fy
        norm = max(1.0, math.hypot(vx, vy))

        standoff_cells = max(1, int(self.frontier_standoff_m / self.latest_map.info.resolution))
        gx = int(round(fx + standoff_cells * vx / norm))
        gy = int(round(fy + standoff_cells * vy / norm))
        return gx, gy

    def find_frontier_goal(self, map_msg, robot_pose):
        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        origin = map_msg.info.origin.position

        if width == 0 or height == 0:
            return None

        grid = np.array(map_msg.data, dtype=np.int16).reshape((height, width))

        robot_x = int((robot_pose.pose.position.x - origin.x) / res)
        robot_y = int((robot_pose.pose.position.y - origin.y) / res)

        if not (0 <= robot_x < width and 0 <= robot_y < height):
            self.get_logger().warn('Robot pose is outside map bounds')
            return None

        if not self.is_cell_traversable(grid[robot_y, robot_x]):
            found = False
            for radius in range(1, 6):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        nx, ny = robot_x + dx, robot_y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if self.is_cell_traversable(grid[ny, nx]):
                                robot_x, robot_y = nx, ny
                                found = True
                                break

                    if found:
                        break

                if found: 
                    break

            if not found:
                self.get_logger().warn('Could not find traversable start cell near robot')
                return None

        reachable_cells = self.get_reachable_cells(grid, width, height, robot_x, robot_y)
        frontier_cells = self.extract_frontier_cells(grid, width, height, reachable_cells)
        
        self.get_logger().info(f'reachable={len(reachable_cells)}, frontier_cells={len(frontier_cells)}')
        
        if not frontier_cells:
            return None

        clusters = self.cluster_frontiers(frontier_cells)
        ranked_clusters = self.rank_frontier_clusters(clusters, robot_x, robot_y)
        
        self.get_logger().info(f'clusters={len(clusters)}, ranked_clusters={len(ranked_clusters)}')
        
        for cluster in ranked_clusters:
            goal_x, goal_y = self.backoff_goal_cell(cluster, robot_x, robot_y)

            if not (0 <= goal_x < width and 0 <= goal_y < height):
                return None
            if not self.is_cell_traversable(grid[goal_y, goal_x]):
                return None

            self.get_logger().info(f'selected cluster size={len(cluster)}, goal=({goal_x},{goal_y})')

            return self.create_pose(goal_x, goal_y, res, origin)

        self.get_logger().warn('Frontier clusters exist, but no valid backed-off goal was found')

    def create_pose(self, mx, my, res, origin):
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = ((mx + 0.5) * res) + origin.x
        pose.pose.position.y = ((my + 0.5) * res) + origin.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose


def main():
    rclpy.init()
    node = ExplorationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.navigator)

    try:
        node.wait_until_ready(executor)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        try:
            node.navigator.cancelTask()
        except Exception:
            pass

        node.navigator.destroy_node()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
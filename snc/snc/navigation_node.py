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

from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty, String, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from snc_interfaces.srv import ExplorationControl
from snc.constants import (
    PATH_EXPLORE_TOPIC,
    PATH_EXPLORE_INTERFACE,
    PATH_EXPLORE_BUFFER_SIZE,
    SNC_STATUS_TOPIC,
    SNC_STATUS_INTERFACE,
    SNC_STATUS_BUFFER_SIZE,
    TRIGGER_HOME_TOPIC,
    TRIGGER_HOME_INTERFACE,
    TRIGGER_HOME_BUFFER_SIZE,
    TRIGGER_START_TOPIC,
    TRIGGER_START_INTERFACE,
    TRIGGER_START_BUFFER_SIZE,
    TRIGGER_TELEOP_TOPIC,
    TRIGGER_TELEOP_INTERFACE,
    TRIGGER_TELEOP_BUFFER_SIZE,
    HAZARD_SIGNAL_TOPIC,
    STARTUP_SYNC_TOPIC,
    STARTUP_SYNC_INTERFACE,
    STARTUP_SYNC_BUFFER_SIZE,
    STARTUP_SYNC_QOS,
    TRIGGER_QOS,
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


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

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

        self.covered = None
        self.last_path_len = 0
        self.latest_path_msg = None
        self.covered_map_info = None
        self.last_processed_cell = None
        self.robot_radius_m = 0.15
        self.safety_margin_m = 0.05
        self.choose_frontier_goal = True

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
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            self.start_callback,
            TRIGGER_QOS
        )

        self.hazard_signal_sub = self.create_subscription(
            Empty,
            HAZARD_SIGNAL_TOPIC,
            self.hazard_signal_callback,
            10
        )

        self.teleop_sub = self.create_subscription(
            TRIGGER_TELEOP_INTERFACE,
            TRIGGER_TELEOP_TOPIC,
            self.teleop_callback,
            TRIGGER_QOS
        )

        self.hazards_sub = self.create_subscription(
            MarkerArray,
            '/hazards',
            self.hazards_callback,
            10
        )

        self.path_tracing_sub = self.create_subscription(
            PATH_EXPLORE_INTERFACE,
            PATH_EXPLORE_TOPIC,
            self.path_explore_callback,
            PATH_EXPLORE_BUFFER_SIZE
        )

        self.status_pub = self.create_publisher(
            SNC_STATUS_INTERFACE, 
            SNC_STATUS_TOPIC, 
            SNC_STATUS_BUFFER_SIZE
        )

        # Startup synchronization publisher - publishes node readiness
        self.pub_startup_sync = self.create_publisher(
            STARTUP_SYNC_INTERFACE,
            STARTUP_SYNC_TOPIC,
            STARTUP_SYNC_QOS
        )

        # Startup synchronization subscriber - waits for all nodes to be ready
        self.sub_startup_sync = self.create_subscription(
            STARTUP_SYNC_INTERFACE,
            STARTUP_SYNC_TOPIC,
            self.startup_sync_callback,
            STARTUP_SYNC_QOS
        )

        # Track which nodes have published readiness
        self.nodes_ready = set()
        self.node_name = self.get_name()  # Use actual node name
        self.all_nodes_ready = False

        self.return_pub = self.create_publisher(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            TRIGGER_QOS
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        self.coverage_marker_pub = self.create_publisher(
            Marker, 
            '/covered_cells_marker',
            1    
        )
        self.coverage_viz_timer = self.create_timer(1.0, self.publish_coverage_marker)

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

        # Set is_ready early so other nodes can proceed with startup sync
        self.is_ready = True
        self.get_logger().info('Exploration node is ready')

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

        self.get_logger().info('Occupancy grid received, ready for exploration')

    def startup_sync_callback(self, msg):
        """Callback for startup synchronization topic. Tracks which nodes have published readiness."""
        if self.all_nodes_ready:
            return
        
        # Extract node name from the message data
        node_name = msg.data if hasattr(msg, 'data') else str(msg)
        if node_name and node_name != self.node_name:
            self.nodes_ready.add(node_name)
            self.get_logger().info(f'Received ready signal from: {node_name}. Ready nodes: {len(self.nodes_ready) + 1}/3')
            
            # Check if all expected nodes are ready (3 nodes: path_tracing, navigation, marker_detection)
            if len(self.nodes_ready) + 1 >= 2:  # +1 for self
                self.all_nodes_ready = True
                self.get_logger().info('All nodes are ready! Starting startup synchronization complete.')
    
    def wait_for_all_nodes_ready(self):
        """Wait for all nodes to publish their readiness on the startup sync topic."""
        self.get_logger().info('Waiting for all nodes to be ready...')
        
        # Publish this node's readiness
        self.pub_startup_sync.publish(String(data=self.node_name))
        
        # Use a separate executor to process callbacks during startup sync
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        
        try:
            while rclpy.ok() and not self.all_nodes_ready:
                executor.spin_once(timeout_sec=0.1)
        finally:
            executor.remove_node(self)
            executor.shutdown()
        
        self.get_logger().info('All nodes ready, proceeding with initialization.')

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
        self.latest_path_msg = msg

        if self.latest_map is None:
            return

        self.ensure_coverage_grid()

        path = msg.poses
        n = len(path)
        
        if n < self.last_path_len:
            self.get_logger().info('Breadcrumb path shrank, rebuilding coverage grid')
            self.rebuild_coverage_from_full_path(path)
            return

        new_points = path[self.last_path_len:]
        for p in new_points:
            cell = self.world_to_map(p.pose.position.x, p.pose.position.y)
            if cell is None:
                continue

            if self.last_processed_cell is None:
                self.paint_disk(cell[0], cell[1])
            else:
                self.paint_segment_covered(self.last_processed_cell, cell)

            self.last_processed_cell = cell

        self.last_path_len = n

    def publish_coverage_marker(self):#
        if self.latest_map is None or self.covered is None:
            return
        
        msg = Marker()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'coverage'
        msg.id = 0
        msg.type = Marker.CUBE_LIST
        msg.action = Marker.ADD
        msg.scale.x = self.latest_map.info.resolution
        msg.scale.y = self.latest_map.info.resolution
        msg.scale.z = 0.01
        msg.pose.orientation.w = 1.0
        msg.frame_locked = True
        
        color = ColorRGBA()
        color.r = 0.0
        color.g = 1.0
        color.b = 1.0
        color.a = 0.7
        msg.color = color
        
        origin = self.latest_map.info.origin.position
        res = self.latest_map.info.resolution
        
        ys, xs = np.where(self.covered)
        for y, x in zip(ys, xs):
            p = Point()
            p.x = origin.x + (x + 0.5) * res
            p.y = origin.y + (y + 0.5) * res
            p.z = 0.01
            msg.points.append(p)
            
        num_points = len(msg.points)
        self.coverage_marker_pub.publish(msg)

    def ensure_coverage_grid(self):
        h = self.latest_map.info.height
        w = self.latest_map.info.width

        map_changed = (
            self.covered is None or
            self.covered.shape != (h, w) or
            self.covered_map_info is None or
            self.covered_map_info.resolution != self.latest_map.info.resolution or
            self.covered_map_info.origin.position.x != self.latest_map.info.origin.position.x or
            self.covered_map_info.origin.position.y != self.latest_map.info.origin.position.y
        )

        if map_changed:
            self.covered = np.zeros((h, w), dtype=bool)
            self.last_path_len = 0
            self.last_processed_cell = None
            self.covered_map_info = self.latest_map.info

            if self.latest_path_msg is not None:
                self.rebuild_coverage_from_full_path(self.latest_path_msg.poses) 

    def world_to_map(self, x, y):
        info = self.latest_map.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)

        if 0 <= mx < info.width and 0 <= my < info.height:
            return (mx, my)

        return None   

    def paint_disk(self, mx, my):
        res = self.latest_map.info.resolution
        radius_cells = max(1, int(math.ceil((self.robot_radius_m * 4) / res)))

        h, w = self.covered.shape
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx**2 + dy**2 > radius_cells**2:
                    continue
                nx, ny = mx + dx, my + dy
                if 0 <= nx < w and 0 <= ny < h:
                    self.covered[ny, nx] = True

    def paint_segment_covered(self, a, b):
        x0, y0 = a
        x1, y1 = b

        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for i in range(steps + 1):
            t = i / steps
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))
            self.paint_disk(x, y)

    def rebuild_coverage_from_full_path(self, path):
        self.ensure_coverage_grid()
        self.covered.fill(False)
        self.last_processed_cell = None

        for p in path:
            cell = self.world_to_map(p.pose.position.x, p.pose.position.y)
            if cell is None:
                continue

            if self.last_processed_cell is None:
                self.paint_disk(cell[0], cell[1])
            else:
                self.paint_segment_covered(self.last_processed_cell, cell)

            self.last_processed_cell = cell
        
        self.last_path_len = len(path)

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

        goal_pose = self.find_navigation_goal(self.latest_map, curr_pose)

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

    def build_safe_free_mask(self, grid):
        res = self.latest_map.info.resolution
        
        inflation_cells = max(
            1, 
            int(math.ceil((self.robot_radius_m + self.safety_margin_m) / res))
        )
        
        height, width = grid.shape
        
        known_free = (grid >= 0) & (grid < MAP_OCCUPIED_THRESHOLD)
        occupied = (grid >= MAP_OCCUPIED_THRESHOLD)
        
        inflated_occupied = np.copy(occupied)
        
        ys, xs = np.where(occupied)
        for y, x in zip(ys, xs):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx**2 + dy**2 > inflation_cells**2:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        inflated_occupied[ny, nx] = True
                    
        safe_free = known_free & (~inflated_occupied)
        return safe_free

    def find_nearby_safe_cell(self, safe_free_mask, start_x, start_y, max_radius=10):
        height, width = safe_free_mask.shape

        for radius in range(0, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = start_x + dx, start_y + dy
                    if 0 <= nx < width and 0 <= ny < height and safe_free_mask[ny, nx]:
                        return nx, ny

        return None, None

    def get_reachable_cells_and_distance(self, safe_free_mask, start_x, start_y):
        height, width = safe_free_mask.shape
        reachable = np.zeros((height, width), dtype=bool)
        dist = np.full((height, width), -1, dtype=np.int32)

        if not safe_free_mask[start_y, start_x]:
            return reachable, dist

        q = collections.deque([(start_x, start_y)])
        reachable[start_y, start_x] = True
        dist[start_y, start_x] = 0

        while q:
            cx, cy = q.popleft()
            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if reachable[ny, nx]:
                    continue
                if not safe_free_mask[ny, nx]:
                    continue

                reachable[ny, nx] = True
                dist[ny, nx] = dist[cy, cx] + 1
                q.append((nx, ny))

        return reachable, dist

    def extract_frontier_cells(self, grid, width, height, reachable_cells_mask):
        frontier_cells = []

        ys, xs = np.where(reachable_cells_mask)
        for y, x in zip(ys, xs):
            if self.cell_has_unknown_neighbours(grid, x, y, width, height):
                frontier_cells.append((x, y))

        return frontier_cells

    def extract_uncovered_cells(self, uncovered_mask):
        ys, xs = np.where(uncovered_mask)
        return [(x, y) for y, x in zip(ys, xs)]

    def rank_clusters(self, clusters, robot_x, robot_y):
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

    def cluster_cells(self, cells):
        frontier_set = set(cells)
        visited = set()
        clusters = []

        for cell in cells:
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

        self.get_logger().info(f'Centroid of chosen cluster: ({centroid_x},{centroid_y})')

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
        
        self.get_logger().info(f'Computed back off goal cell based on centroid and standoff ({self.frontier_standoff_m}): ({gx},{gy})')

        return gx, gy

    def choose_coverage_goal_from_cluster(self, cluster, dists):
        best_cell = None
        best_dist = -1

        for x, y in cluster:
            d = dists[y, x]
            if d > best_dist:
                best_dist = d
                best_cell = (x, y)

        return best_cell

    def find_navigation_goal(self, map_msg, robot_pose):
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

        safe_free_mask = self.build_safe_free_mask(grid)
        if not safe_free_mask[robot_y, robot_x]:
            robot_x, robot_y = self.find_nearby_safe_cell(safe_free_mask, robot_x, robot_y)
            if robot_x is None:
                self.get_logger().warn('Robot is not on a safe cell and no nearby safe cell was found')
                return None

        reachable_cells, dists = self.get_reachable_cells_and_distance(safe_free_mask, robot_x, robot_y)
        
        free_mask = np.vectorize(self.is_cell_traversable)(grid)
        reachable_mask = reachable_cells
        covered_mask = self.covered if self.covered is not None else np.zeros_like(reachable_mask, dtype=bool)
        uncovered_mask = reachable_mask & free_mask & (~covered_mask)

        num_free = np.count_nonzero(free_mask)
        num_reachable = np.count_nonzero(reachable_mask)
        num_covered = np.count_nonzero(covered_mask)
        num_uncovered = np.count_nonzero(uncovered_mask)
        reachable_covered_mask = reachable_mask & covered_mask
        covered_pct = (100.0 * num_covered / num_free) if num_free > 0 else 0.0
        reachable_covered_pct = (
            100.0 * (num_reachable - num_uncovered) / num_reachable
            if num_reachable > 0 else 0.0
        )

        self.get_logger().info(
            f"free={num_free}, reachable={num_reachable}, covered={num_covered}, "
            f"uncovered={num_uncovered}, covered/free={covered_pct:.1f}%, "
            f"covered/reachable={reachable_covered_pct:.1f}%, "
            f"reachable_covered={np.count_nonzero(reachable_covered_mask)}"
        )

        if self.choose_frontier_goal:
            self.get_logger().info('Choosing frontier goal...')
            frontier_goal = self.find_frontier_goal(grid, safe_free_mask, width, height, robot_x, robot_y, reachable_mask, origin, res)
            if frontier_goal is not None:
                return frontier_goal
            else:
                self.get_logger().info('No frontier goal found. Choosing coverage goal instead...')
                coverage_goal = self.find_coverage_goal(safe_free_mask, width, height, robot_x, robot_y, dists, uncovered_mask, origin, res)
                if coverage_goal is not None:
                    return coverage_goal
        else:
            self.get_logger().info('Choosing coverage goal...')
            coverage_goal = self.find_coverage_goal(safe_free_mask, width, height, robot_x, robot_y, dists, uncovered_mask, origin, res)
            if coverage_goal is not None:
                return coverage_goal
            else:
                self.get_logger().info('No coverage goal found. Choosing frontier goal instead...')
                frontier_goal = self.find_frontier_goal(grid, safe_free_mask, width, height, robot_x, robot_y, reachable_mask, origin, res)
                if frontier_goal is not None:
                    return frontier_goal

        self.get_logger().info('No more frontiers/uncovered cells found. Maze fully explored')
        return None

    def find_coverage_goal(self, safe_free_mask, width, height, robot_x, robot_y, dists, uncovered_mask, origin, res):
        uncovered_cells = self.extract_uncovered_cells(uncovered_mask)
        
        if not uncovered_cells:
            self.get_logger().warn('No uncovered cells found')
            return None

        uncovered_clusters = self.cluster_cells(uncovered_cells)
        ranked_clusters = self.rank_clusters(uncovered_clusters, robot_x, robot_y)

        if ranked_clusters is None or len(ranked_clusters) <= 0:
            self.get_logger().warn('No uncovered clusters found')
            return None

        self.get_logger().info(f'#uncovered clusters={len(uncovered_clusters)}, #ranked_uncovered_clusters={len(ranked_clusters)}')

        for cluster in ranked_clusters:
            #goal_x, goal_y = self.backoff_goal_cell(cluster, robot_x, robot_y)
            goal_x, goal_y = self.choose_coverage_goal_from_cluster(cluster, dists)

            if not (0 <= goal_x < width and 0 <= goal_y < height):
                continue
            if not safe_free_mask[goal_y, goal_x]:
                continue

            self.get_logger().info(f'selected uncovered cluster size={len(cluster)}, goal=({goal_x},{goal_y})')

            return self.create_pose(goal_x, goal_y, res, origin)

        self.get_logger().warn('Uncovered clusters exist, but no valid goal was found')
        return None

    def find_frontier_goal(self, grid, safe_free_mask, width, height, robot_x, robot_y, reachable_mask, origin, res):
        frontier_cells = self.extract_frontier_cells(grid, width, height, reachable_mask)
        
        if not frontier_cells:
            self.get_logger().warn('No frontier cells found')
            return None

        frontier_clusters = self.cluster_cells(frontier_cells)
        ranked_clusters = self.rank_clusters(frontier_clusters, robot_x, robot_y)
        
        if ranked_clusters is None or len(ranked_clusters) <= 0:
            self.get_logger().warn('No frontier clusters found')
            return None

        self.get_logger().info(f'frontier clusters={len(frontier_clusters)}, ranked_frontier_clusters={len(ranked_clusters)}')
        
        for cluster in ranked_clusters:
            goal_x, goal_y = self.backoff_goal_cell(cluster, robot_x, robot_y)

            if not (0 <= goal_x < width and 0 <= goal_y < height):
                continue
            if not safe_free_mask[goal_y, goal_x]:
                continue

            self.get_logger().info(f'selected frontier cluster size={len(cluster)}, goal=({goal_x},{goal_y})')

            return self.create_pose(goal_x, goal_y, res, origin)

        self.get_logger().warn('Frontier clusters exist, but no valid backed-off goal was found')
        return None
        
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
    node = NavigationNode()

    # Wait for all nodes to be ready before starting
    #node.wait_for_all_nodes_ready()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        node.wait_until_ready(executor)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        try:
            node.navigator.cancelTask()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

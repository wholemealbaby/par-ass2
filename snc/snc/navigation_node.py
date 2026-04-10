#!/usr/bin/env python3

import collections
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Costmap constants
INSCRIBED = 253
LETHAL = 254
UNKNOWN = 255

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        self.navigator = BasicNavigator()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.global_frame = 'map'
        self.robot_base_frame = 'base_link'
        
        self.declare_parameter('planner_frequency', 1.0)
        self.planner_frequency = self.get_parameter('planner_frequency').value
        
        self.plan_timer = None
        self.goal_active = False
        self.exploration_done = False
        self.no_frontier_count = 0
        self.no_frontier_limit = 5
        self.get_logger().info('Navigation node created')

    def wait_until_ready(self, executor: MultiThreadedExecutor):
        self.get_logger().info('Waiting for Nav2 to become active...')
        
        self.navigator.waitUntilNav2Active(localizer='slam_toolbox')
        self.get_logger().info('Nav2 is active')
        
        self.get_logger().info(f'Waiting for TF {self.global_frame} -> {self.robot_base_frame}...')
        
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
        
        self.get_logger().info('Waiting for global costmap...')
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            try:
                costmap = self.navigator.getGlobalCostmap()
                if (
                    costmap is not None
                    and costmap.metadata.size_x > 0
                    and costmap.metadata.size_y > 0
                    and len(costmap.data) == costmap.metadata.size_x * costmap.metadata.size_y    
                ):
                    break
            except Exception:
                pass
            
        self.get_logger().info('Global costmap is available')
        self.get_logger().info('All required nodes/information are ready. Start exploration.')
        
    def is_cell_traversable(self, cost):
        return 0 <= int(cost) < INSCRIBED

    def cell_has_unknown_neighbours(self, grid, x, y, width, height):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy

                if 0 <= nx < width and 0 <= ny < height:
                    if(int(grid[ny, nx]) == UNKNOWN):
                        return True
        return False

    def start_exploration(self):
        period = 1.0 / float(self.planner_frequency)
        self.plan_timer = self.create_timer(period, self.plan_step)
        self.plan_step()
        
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.2),    
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not get robot pose yet: {ex}')
            return None
        
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        
        return pose
    
    def plan_step(self):
        if self.exploration_done:
            return
        
        # If a goal is still active running, do not send a new one yet
        if self.goal_active:
            if not self.navigator.isTaskComplete():
                return
            
            result = self.navigator.getResult()
            self.goal_active = False
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Fronier reached, replanning...')
            else:
                self.get_logger().warn(f'Navigation ended with result: {result}, replanning...')
                
        curr_pose = self.get_robot_pose()
        if curr_pose is None:
            return
        
        try:
            costmap = self.navigator.getGlobalCostmap()
        except Exception as ex:
            self.get_logger().warn(f'Could not read global costmap: {ex}')
            return
        
        goal_pose = self.find_frontier_goal(costmap, curr_pose)
        
        if goal_pose is None:
            self.no_frontier_count += 1
            self.get_logger().info(f'No frontier found this cycle ({self.no_frontier_count}/{self.no_frontier_limit})')

            if self.no_frontier_count >= self.no_frontier_limit:
                self.get_logger().info('Mapping finished')
                self.exploration_done = True
                if self.plan_timer is not None:
                    self.plan_timer.cancel()

                return
        
        self.get_logger().info(f'Sending frontier goal: x={goal_pose.pose.position.x:.2f}, '
                               f'y={goal_pose.pose.position.y:.2f}'
        )
        self.navigator.goToPose(goal_pose)
        self.goal_active = True
        
    def find_frontier_goal(self, costmap_msg, robot_pose):
        width = costmap_msg.metadata.size_x
        height = costmap_msg.metadata.size_y
        res = costmap_msg.metadata.resolution
        origin = costmap_msg.metadata.origin.position
        
        # Invalid costmap size --> cannot find frontier goal
        if width == 0 or height == 0:
            return None

        # Reshape the flat 1D data into a grid
        grid = np.array(costmap_msg.data).reshape((height, width))
        
        # Convert robot world coordinates to map indices
        start_x = int((robot_pose.pose.position.x - origin.x) / res)
        start_y = int((robot_pose.pose.position.y - origin.y) / res)
        
        if not (0 <= start_x < width and 0 <= start_y < height):
            self.get_logger().warn('Robot pose is outside the global costmap bounds')
            return None

        if not self.cell_has_unknown_neighbours(grid, start_x, start_y, width, height):
            found = False
            for radius in range(1, 6):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        nx, ny = start_x + dx, start_y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            cost = grid[ny, nx]
                            if self.is_cell_traversable(cost):
                                found = True
                                break
                    if found == True:
                        break
                if found == True:
                    break
            if not found:
                self.get_logger().warn('Could not find traversable start cell near robot')

        queue = collections.deque([(start_x, start_y)])
        visited = set([(start_x, start_y)])
        
        while queue:
            curr_x, curr_y = queue.popleft()
            
            # Only expand through free cells
            if not self.is_cell_traversable(grid[curr_y, curr_x]):
                continue

            if self.cell_has_unknown_neighbours(grid, curr_x, curr_y, width, height):
                return self.create_pose(curr_x, curr_y, res, origin)
            
            # Check 4-way neighbours (up, down, left, right)
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = curr_x + dx, curr_y + dy
                
                # Neighbour outside global costmap bounds
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                # Neighbour already visited
                if (nx, ny) in visited:
                    continue
                if not self.is_cell_traversable(grid[ny, nx]):
                    continue

                visited.add((nx, ny))
                queue.append((nx, ny))
                        
        return None # No frontiers left (map is complete)
    
    def create_pose(self, mx, my, res, origin):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        
        # Convert map index back to world coordinates
        # use cell center
        pose.pose.position.x = ((mx + 0.5) * res) + origin.x
        pose.pose.position.y = ((my + 0.5) * res) + origin.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        return pose

def main():
    rclpy.init()
    node = NavigationNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.navigator)
    
    try:
        node.wait_until_ready(executor)
        node.start_exploration()
        
        while rclpy.ok() and not node.exploration_done:
            executor.spin_once(timeout_sec=0.1)
            
    finally:
        if node.plan_timer is not None:
            node.plan_timer.cancel()
            
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
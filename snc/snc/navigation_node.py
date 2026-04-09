#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from constants import (
    PATH_EXPLORE_BUFFER_SIZE,
    PATH_RETURN_BUFFER_SIZE,
    HOME_TRIGGER_BUFFER_SIZE,
    HOME_TRIGGER_TOPIC,
    PATH_EXPLORE_TOPIC,
    PATH_RETURN_TOPIC
)
class PathTracingNode(Node):
    def __init__(self):
        super().__init__('path_tracing_node')
        self.get_logger().info('Path tracing node launched')
        # Add a timer to show it's alive
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        # Return home trigger
        self.sub_home_trigger = self.create_subscription(
            Empty,
            HOME_TRIGGER_TOPIC,
            self.home_trigger_callback,
            HOME_TRIGGER_BUFFER_SIZE
        )

        # Path exploration subscription for waypoints
        self.sub_path_explorer = self.create_subscription(
            Path,
            PATH_EXPLORE_TOPIC,
            self.path_explorer_callback,
            PATH_EXPLORE_BUFFER_SIZE
        )

        # Publisher for return waypoints
        self.pub_path_return = self.create_publisher(
            Path,
            PATH_RETURN_TOPIC,
            PATH_RETURN_BUFFER_SIZE
        )

        self.path_return = []

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Path tracing node spinning ({self.counter})')

    def home_trigger_callback(self, msg):
        self.get_logger().info('Home trigger received, starting path tracing')

def main():
    rclpy.init()
    node = PathTracingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
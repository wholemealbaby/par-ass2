#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation node launched')
        # Add a timer to show it's alive
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Navigation node spinning ({self.counter})')


def main():
    rclpy.init()
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PathTracingNode()

    # Pre-flight check
    # Wait max 10 secs for the robot to appear in the TF tree
    node.get_logger().info("Performing pre-flight TF check...")
    ready = node.tf_buffer.wait_for_transform_async('map', 'base_link', rclpy.time.Time())
    
    # Keep node responsive while waiting for transform
    # Abort on keyboard interrupt
    try:
        rclpy.spin_until_future_complete(node, ready, timeout_sec=10.0)

        if ready.done():
            node.get_logger().info("Robot pose detected. Starting recorder.")
            rclpy.spin(node)
        else:
            node.get_logger().error("Timeout: Could not find robot pose. Is the remote robot running?")
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        self.get_logger().info('Marker detection node launched')
        # Add a timer to show it's alive
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Marker detection node spinning ({self.counter})')


def main():
    rclpy.init()
    node = MarkerDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
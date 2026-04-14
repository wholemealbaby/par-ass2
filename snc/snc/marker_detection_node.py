#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from constants import (
    START_CHALLENGE_INTERFACE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_BUFFER_SIZE,
)

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        self.get_logger().info('Marker detection node launched')
        
        
        # Create publisher to /snc_start topic
        self.pub_start_challenge(
            START_CHALLENGE_INTERFACE,
            START_CHALLENGE_TOPIC,
            START_CHALLENGE_BUFFER_SIZE,
        )


    def timer_callback(self):
        pass
        

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

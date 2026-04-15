#!/usr/bin/env python3
"""
Test for the marker detection node (Node #2 - Hazard Marker Detection).
This test verifies that the marker detection node correctly detects the "Start" object
and publishes to the /start topic.

The test:
1. Publishes a Float32MultiArray message to /objects with a "Start" object
2. Subscribes to /start_challenge to verify the detection triggers the challenge start
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty
import time
import sys
import os

# Add the snc module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from snc.constants import (
    OBJECTS_TOPIC,
    OBJECTS_BUFFER_SIZE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_BUFFER_SIZE,
    HAZARD_IMAGE_MAP
)


class DetectStartTest(Node):
    """Test node to verify Start object detection in marker detection node."""
    
    def __init__(self):
        super().__init__('detect_start_test')
        
        # Track test state
        self.start_challenge_received = False
        self.start_challenge_count = 0
        self.test_passed = False
        self.test_failed = False
        self.test_timeout = False
        
        # Test timing
        self.start_time = time.time()
        self.test_duration = 10.0  # 10 seconds timeout
        
        # Subscribe to /start_challenge topic to verify detection
        self.start_challenge_sub = self.create_subscription(
            Empty,
            START_CHALLENGE_TOPIC,
            self.start_challenge_callback,
            START_CHALLENGE_BUFFER_SIZE
        )
        
        # Publisher for /objects topic
        self.objects_pub = self.create_publisher(
            Float32MultiArray,
            OBJECTS_TOPIC,
            OBJECTS_BUFFER_SIZE
        )
        
        self.get_logger().info('Detect Start Test initialized')
        self.get_logger().info(f'Subscribed to {START_CHALLENGE_TOPIC}')
        self.get_logger().info(f'Will publish to {OBJECTS_TOPIC}')
        
        # Timer to trigger the test after a short delay
        self.timer = self.create_timer(1.0, self.publish_start_object)
        
    def start_challenge_callback(self, msg):
        """Callback for /start_challenge messages."""
        self.start_challenge_count += 1
        self.start_challenge_received = True
        self.get_logger().info(f'Received /start_challenge message (count: {self.start_challenge_count})')
        
        # Test passes when we receive the start challenge
        if self.start_challenge_count >= 1:
            self.test_passed = True
            self.get_logger().info('TEST PASSED: Start object detected successfully!')
            
    def publish_start_object(self):
        """Publish a Float32MultiArray message with a "Start" object."""
        # Get the object ID for "Start" from HAZARD_IMAGE_MAP
        start_object_id = HAZARD_IMAGE_MAP.get("Start")
        if start_object_id is None:
            self.get_logger().error('ERROR: "Start" not found in HAZARD_IMAGE_MAP')
            self.test_failed = True
            return
            
        self.get_logger().info(f'Publishing Start object with ID: {start_object_id}')
        
        # Create a Float32MultiArray message representing one object
        # Each object has 12 fields:
        # [0] object_id (int as float)
        # [1] width
        # [2] height
        # [3-8] rotation matrix (3x3, row-major, first 6 elements)
        # [9] dx (x position)
        # [10] dy (y position)
        # [11] confidence
        
        # Identity rotation matrix (first 6 elements): [1, 0, 0, 1, 0, 0]
        # Position at center: dx=0, dy=0
        # High confidence: 0.99
        data = [
            float(start_object_id),  # object_id
            1.0,                     # width
            1.0,                     # height
            1.0, 0.0, 0.0,           # rotation matrix row 1 (h11, h12, h13)
            0.0, 1.0, 0.0,           # rotation matrix row 2 (h21, h22, h23)
            0.0, 0.0,                # rotation matrix row 3 (h31, h32)
            0.0,                     # dx (x position)
            0.0,                     # dy (y position)
            0.99                     # confidence
        ]
        
        msg = Float32MultiArray()
        msg.data = data
        
        # Publish the message
        self.objects_pub.publish(msg)
        self.get_logger().info(f'Published Start object message: {data}')
        
        # Cancel the timer after publishing
        if not self.timer.cancel():
            self.timer.destroy()
            
    def check_test_status(self):
        """Check if test has passed or timed out."""
        elapsed_time = time.time() - self.start_time
        
        if self.test_passed:
            self.get_logger().info(f'Test completed successfully in {elapsed_time:.2f} seconds')
            return True
            
        if elapsed_time > self.test_duration:
            self.test_timeout = True
            self.get_logger().error(f'TEST FAILED: Timeout after {self.test_duration} seconds')
            self.get_logger().error(f'Start challenge received: {self.start_challenge_count} times')
            return True
            
        return False


def main():
    """Main function to run the test."""
    rclpy.init()
    
    test_node = DetectStartTest()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(test_node)
            
            # Check test status
            if test_node.check_test_status():
                break
                
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
        
    finally:
        # Print test result
        if test_node.test_passed:
            print("\n" + "="*60)
            print("TEST RESULT: PASSED")
            print("="*60)
            print(f"Start object (ID: {HAZARD_IMAGE_MAP.get('Start')}) was successfully detected")
            print(f"Marker detection node published to {START_CHALLENGE_TOPIC}")
            print("="*60)
            test_node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        elif test_node.test_failed:
            print("\n" + "="*60)
            print("TEST RESULT: FAILED")
            print("="*60)
            print("Test failed due to error")
            print("="*60)
            test_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        elif test_node.test_timeout:
            print("\n" + "="*60)
            print("TEST RESULT: FAILED (TIMEOUT)")
            print("="*60)
            print(f"Did not receive /start_challenge within {test_node.test_duration} seconds")
            print("="*60)
            test_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        else:
            print("\n" + "="*60)
            print("TEST RESULT: UNKNOWN")
            print("="*60)
            test_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)


if __name__ == '__main__':
    main()

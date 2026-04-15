#!/usr/bin/env python3
"""
Test for the marker detection node (Node #2 - Hazard Marker Detection).
This test verifies that the marker detection node correctly detects the "Start" object
and publishes to the /start topic.

The test:
1. Publishes an ObjectsStamped message to /objectsStamped with a "Start" object
2. Subscribes to /start_challenge to verify the detection triggers the challenge start
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Header
import time
import sys
import os

# Add the snc module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from snc.constants import (
    OBJECTS_TOPIC,
    OBJECTS_BUFFER_SIZE,
    OBJECTS_INTERFACE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_BUFFER_SIZE,
    OBJECT_MAP
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
        
        # Publisher for /objectsStamped topic
        self.objects_pub = self.create_publisher(
            OBJECTS_INTERFACE,
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
        """Publish an ObjectsStamped message with a "Start" object."""
        # Get the object ID for "Start" from OBJECT_MAP
        start_object_id = OBJECT_MAP.get("Start")
        if start_object_id is None:
            self.get_logger().error('ERROR: "Start" not found in OBJECT_MAP')
            self.test_failed = True
            return
            
        self.get_logger().info(f'Publishing Start object with ID: {start_object_id}')
        
        # Create an ObjectsStamped message
        msg = OBJECTS_INTERFACE()
        msg.objects.data = [
            float(start_object_id), # ID
            428.0, 417.0,           # Width, Height
            0.58, 0.03, 0.0,        # H11, H12, H13
            -0.02, 0.58, 0.0,       # H21, H22, H23
            234.9, -25.6, 1.0       # H31, H32, H33
        ]
        
        # Add header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera"
        msg.header = header
        
        # Publish the message
        self.objects_pub.publish(msg)
        self.get_logger().info(f'Published Start object message with header')
        
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
            print(f"Start object (ID: {OBJECT_MAP.get('Start')}) was successfully detected")
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

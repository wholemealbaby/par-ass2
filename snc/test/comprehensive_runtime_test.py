#!/usr/bin/env python3
"""
Comprehensive runtime tests for the path tracing node.
This script performs more detailed validation of the path tracing node behavior
based on the ROS2 Search & Navigation Challenge specification.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped
import time
import sys
import os
import math

# Add the snc module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from snc.constants import (
    PATH_EXPLORE_TOPIC,
    PATH_RETURN_TOPIC,
    HOME_TRIGGER_TOPIC,
    PATH_EXPLORE_BUFFER_SIZE,
    PATH_RETURN_BUFFER_SIZE
)

class ComprehensivePathTracingTest(Node):
    def __init__(self):
        super().__init__('comprehensive_path_tracing_test')
        
        # Track received messages and state
        self.explore_path_count = 0
        self.return_path_count = 0
        self.home_trigger_count = 0
        self.status_messages = []
        
        # Store latest messages
        self.latest_explore_path = None
        self.latest_return_path = None
        
        # State tracking
        self.paths_have_waypoints = False
        self.paths_have_correct_frame = True
        self.paths_are_valid = True
        
        # Subscribe to path topics
        self.explore_sub = self.create_subscription(
            Path,
            PATH_EXPLORE_TOPIC,
            self.explore_path_callback,
            PATH_EXPLORE_BUFFER_SIZE
        )
        
        self.return_sub = self.create_subscription(
            Path,
            PATH_RETURN_TOPIC,
            self.return_path_callback,
            PATH_RETURN_BUFFER_SIZE
        )
        
        # Subscribe to home trigger
        self.home_trigger_sub = self.create_subscription(
            Empty,
            HOME_TRIGGER_TOPIC,
            self.home_trigger_callback,
            1
        )
        
        # Subscribe to status topic (mentioned in spec)
        self.status_sub = self.create_subscription(
            String,
            '/snc_status',
            self.status_callback,
            10
        )
        
        # Test duration and timeout
        self.test_duration = 45.0  # seconds
        self.start_time = time.time()
        
        self.get_logger().info('Comprehensive path tracing test initialized')
        self.get_logger().info(f'Subscribed to topics:')
        self.get_logger().info(f'  - {PATH_EXPLORE_TOPIC}')
        self.get_logger().info(f'  - {PATH_RETURN_TOPIC}')
        self.get_logger().info(f'  - {HOME_TRIGGER_TOPIC}')
        self.get_logger().info(f'  - /snc_status')

    def explore_path_callback(self, msg):
        """Callback for explore path messages"""
        self.explore_path_count += 1
        self.latest_explore_path = msg
        
        # Validate path structure
        if not self._validate_path_structure(msg, 'explore'):
            self.paths_are_valid = False
            
        if len(msg.poses) > 0:
            self.paths_have_waypoints = True
            
        self.get_logger().info(f'Explore path #{self.explore_path_count}: {len(msg.poses)} waypoints')

    def return_path_callback(self, msg):
        """Callback for return path messages"""
        self.return_path_count += 1
        self.latest_return_path = msg
        
        # Validate path structure
        if not self._validate_path_structure(msg, 'return'):
            self.paths_are_valid = False
            
        if len(msg.poses) > 0:
            self.paths_have_waypoints = True
            
        self.get_logger().info(f'Return path #{self.return_path_count}: {len(msg.poses)} waypoints')

    def _validate_path_structure(self, path_msg, path_type):
        """Validate the structure of a path message"""
        # Check frame ID
        if path_msg.header.frame_id != 'map':
            self.get_logger().error(f'{path_type.capitalize()} path frame is incorrect: {path_msg.header.frame_id}')
            return False
            
        # Check that we have valid poses
        if len(path_msg.poses) == 0:
            self.get_logger().warn(f'{path_type.capitalize()} path has no waypoints')
            return True  # Not necessarily an error, paths can be empty initially
            
        # Validate each pose
        for i, pose in enumerate(path_msg.poses):
            # Check for valid position
            if not isinstance(pose.pose.position.x, (int, float)) or \
               not isinstance(pose.pose.position.y, (int, float)):
                self.get_logger().error(f'{path_type.capitalize()} path pose {i} has invalid position')
                return False
                
            # Check for valid orientation (quaternion)
            if not isinstance(pose.pose.orientation.x, (int, float)) or \
               not isinstance(pose.pose.orientation.y, (int, float)) or \
               not isinstance(pose.pose.orientation.z, (int, float)) or \
               not isinstance(pose.pose.orientation.w, (int, float)):
                self.get_logger().error(f'{path_type.capitalize()} path pose {i} has invalid orientation')
                return False
                
        return True

    def home_trigger_callback(self, msg):
        """Callback for home trigger messages"""
        self.home_trigger_count += 1
        self.get_logger().info(f'Home trigger #{self.home_trigger_count} received')

    def status_callback(self, msg):
        """Callback for status messages"""
        self.status_messages.append(msg.data)
        self.get_logger().info(f'Status: {msg.data}')

    def validate_comprehensive(self):
        """Perform comprehensive validation of path tracing behavior"""
        self.get_logger().info('Performing comprehensive validation...')
        
        # Basic checks
        self.get_logger().info('\n--- BASIC VALIDATION ---')
        
        # Check if path topics are being published
        if self.explore_path_count > 0:
            self.get_logger().info(f'✓ Explore paths are being published: {self.explore_path_count} total')
            if self.latest_explore_path:
                self.get_logger().info(f'  Latest has {len(self.latest_explore_path.poses)} waypoints')
        else:
            self.get_logger().info('ℹ No explore paths published yet')
            
        if self.return_path_count > 0:
            self.get_logger().info(f'✓ Return paths are being published: {self.return_path_count} total')
            if self.latest_return_path:
                self.get_logger().info(f'  Latest has {len(self.latest_return_path.poses)} waypoints')
        else:
            self.get_logger().info('ℹ No return paths published yet')
            
        # Check if home trigger was received (necessary for return path generation)
        if self.home_trigger_count > 0:
            self.get_logger().info(f'✓ Home trigger received: {self.home_trigger_count} times')
        else:
            self.get_logger().info('ℹ No home trigger received yet (expected before return path generation)')
            
        # Check path frames
        self.get_logger().info('\n--- FRAME VALIDATION ---')
        if self.latest_explore_path:
            if self.latest_explore_path.header.frame_id == 'map':
                self.get_logger().info('✓ Explore path frame is correct (map)')
            else:
                self.get_logger().error(f'✗ Explore path frame incorrect: {self.latest_explore_path.header.frame_id}')
                self.paths_have_correct_frame = False
                
        if self.latest_return_path:
            if self.latest_return_path.header.frame_id == 'map':
                self.get_logger().info('✓ Return path frame is correct (map)')
            else:
                self.get_logger().error(f'✗ Return path frame incorrect: {self.latest_return_path.header.frame_id}')
                self.paths_have_correct_frame = False
                
        # Check for waypoints
        self.get_logger().info('\n--- WAYPOINT VALIDATION ---')
        if self.paths_have_waypoints:
            self.get_logger().info('✓ At least one path has waypoints (indicating path tracing is working)')
        else:
            self.get_logger().warn('ℹ No waypoints observed yet')
            
        # Check status messages
        self.get_logger().info('\n--- STATUS VALIDATION ---')
        if self.status_messages:
            self.get_logger().info(f'✓ Status messages received: {len(self.status_messages)}')
            for i, status in enumerate(self.status_messages[:3]):  # Show first 3
                self.get_logger().info(f'  Status {i+1}: {status}')
            if len(self.status_messages) > 3:
                self.get_logger().info(f'  ... and {len(self.status_messages) - 3} more')
        else:
            self.get_logger().info('ℹ No status messages received yet')
            
        # Overall assessment
        self.get_logger().info('\n--- OVERALL ASSESSMENT ---')
        
        success = True
        
        if not self.paths_have_correct_frame:
            self.get_logger().error('✗ Critical: Paths are not published in map frame')
            success = False
        elif self.paths_have_waypoints:
            self.get_logger().info('✓ Paths appear to be correctly structured and published in map frame')
        else:
            self.get_logger().info('ℹ Paths may be correctly configured but no waypoints recorded yet')
            
        if not self.paths_are_valid:
            self.get_logger().error('✗ Critical: Invalid path structures detected')
            success = False
        elif self.explore_path_count > 0 or self.return_path_count > 0:
            self.get_logger().info('✓ Path structures appear valid')
            
        return success

    def run_comprehensive_test(self):
        """Run the comprehensive runtime test for specified duration"""
        self.get_logger().info(f'Starting comprehensive runtime test for {self.test_duration} seconds...')
        
        start_time = time.time()
        last_progress_update = start_time
        
        while time.time() - start_time < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Progress update every 10 seconds
            current_time = time.time()
            if current_time - last_progress_update >= 10:
                elapsed = int(current_time - start_time)
                self.get_logger().info(f'Test running for {elapsed}s...')
                last_progress_update = current_time
                
        # Final validation
        self.get_logger().info('Test period completed. Performing final comprehensive validation...')
        success = self.validate_comprehensive()
        
        # Final summary
        self.get_logger().info('\n=== COMPREHENSIVE TEST SUMMARY ===')
        self.get_logger().info(f'Explore paths: {self.explore_path_count}')
        self.get_logger().info(f'Return paths: {self.return_path_count}')
        self.get_logger().info(f'Home triggers: {self.home_trigger_count}')
        self.get_logger().info(f'Status messages: {len(self.status_messages)}')
        self.get_logger().info(f'Test duration: {self.test_duration}s')
        
        if success:
            self.get_logger().info('✓ Comprehensive test completed successfully')
        else:
            self.get_logger().info('✗ Comprehensive test had validation issues')
            
        return success


def main():
    rclpy.init()
    
    # Create test node
    test_node = ComprehensivePathTracingTest()
    
    try:
        # Run the test
        success = test_node.run_comprehensive_test()
        
        # Clean shutdown
        test_node.destroy_node()
        rclpy.shutdown()
        
        # Return exit code based on success
        return 0 if success else 1
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
        test_node.destroy_node()
        rclpy.shutdown()
        return 1


if __name__ == '__main__':
    sys.exit(main())
#!/usr/bin/env python3
"""
Runtime tests for the path tracing node.
This script monitors the topic output of the path tracing node
and validates if the node is performing as expected based on the spec.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Empty
import time
import sys
import os
import signal
import threading
from datetime import datetime
from pathlib import Path
import logging

# Add the snc module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from snc.constants import (
    PATH_EXPLORE_TOPIC,
    PATH_EXPLORE_BUFFER_SIZE,
    PATH_EXPLORE_INTERFACE,
    PATH_RETURN_TOPIC,
    PATH_RETURN_BUFFER_SIZE,
    PATH_RETURN_INTERFACE,
    TRIGGER_HOME_TOPIC,
    TRIGGER_HOME_BUFFER_SIZE,
    TRIGGER_HOME_INTERFACE,
    TRIGGER_START_TOPIC,
    TRIGGER_START_BUFFER_SIZE,
    TRIGGER_START_INTERFACE,
    STARTUP_SYNC_TOPIC,
    STARTUP_SYNC_BUFFER_SIZE,
    STARTUP_SYNC_INTERFACE,
    TRIGGER_QOS,
    TEST_SYNC_CHECK_TIMEOUT
)

class PathTracingRuntimeTest(Node):
    def __init__(self):
        super().__init__('path_tracing_runtime_test')
        
        # Track received messages
        self.explore_path_count = 0
        self.return_path_count = 0
        self.home_trigger_count = 0
        
        # Store latest messages
        self.latest_explore_path = None
        self.latest_return_path = None
        
        # Track startup sync signals from other nodes
        self.startup_sync_signals = {}
        self.expected_nodes = {'path_tracing', 'navigation', 'marker_detection'}
        self.all_nodes_ready = False
        
        # Publisher for trigger_start topic
        self.pub_trigger_start = self.create_publisher(
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            TRIGGER_QOS
        )
        
        # Subscriber for startup sync topic
        self.sub_startup_sync = self.create_subscription(
            STARTUP_SYNC_INTERFACE,
            STARTUP_SYNC_TOPIC,
            self.startup_sync_callback,
            STARTUP_SYNC_BUFFER_SIZE
        )
        
        # Subscribe to path topics
        self.explore_sub = self.create_subscription(
            PATH_EXPLORE_INTERFACE,
            PATH_EXPLORE_TOPIC,
            self.explore_path_callback,
            PATH_EXPLORE_BUFFER_SIZE
        )
        
        self.return_sub = self.create_subscription(
            PATH_RETURN_INTERFACE,
            PATH_RETURN_TOPIC,
            self.return_path_callback,
            PATH_RETURN_BUFFER_SIZE
        )
        
        # Subscribe to home trigger
        self.home_trigger_sub = self.create_subscription(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            self.home_trigger_callback,
            TRIGGER_HOME_BUFFER_SIZE
        )
        
        # Test duration and timeout
        self.test_duration = TEST_SYNC_CHECK_TIMEOUT  # seconds
        self.start_time = time.time()
        
        # Flag to indicate if the test was interrupted
        self.interrupted = False
        
        # Setup logging files with timestamps
        self.log_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file_path = os.path.join(self.log_dir, f'path_tracing_trigger_topics_{timestamp}.log')
        self.summary_file_path = os.path.join(self.log_dir, f'path_tracing_trigger_summary_{timestamp}.log')
        
        # Setup file logger
        self.file_logger = logging.getLogger(f'file_logger_{id(self)}')
        self.file_logger.setLevel(logging.DEBUG)
        file_handler = logging.FileHandler(self.log_file_path)
        file_handler.setLevel(logging.DEBUG)
        file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        file_handler.setFormatter(file_formatter)
        self.file_logger.addHandler(file_handler)
        
        self.file_logger.info('=' * 60)
        self.file_logger.info('PATH TRACING RUNTIME TEST - INITIALIZING')
        self.file_logger.info('=' * 60)
        self.file_logger.info(f'Subscribed to topics:')
        self.file_logger.info(f'  - {PATH_EXPLORE_TOPIC}')
        self.file_logger.info(f'  - {PATH_RETURN_TOPIC}')
        self.file_logger.info(f'  - {TRIGGER_HOME_TOPIC}')
        self.file_logger.info(f'  - {TRIGGER_START_TOPIC}')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('PATH TRACING RUNTIME TEST - INITIALIZING')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Subscribed to topics:')
        self.get_logger().info(f'  - {PATH_EXPLORE_TOPIC}')
        self.get_logger().info(f'  - {PATH_RETURN_TOPIC}')
        self.get_logger().info(f'  - {TRIGGER_HOME_TOPIC}')
        self.get_logger().info(f'  - {TRIGGER_START_TOPIC}')
        self.get_logger().info(f'Log files will be saved to: {self.log_dir}')

    def explore_path_callback(self, msg):
        """Callback for explore path messages"""
        self.explore_path_count += 1
        self.latest_explore_path = msg
        message = f'Received explore path #{self.explore_path_count} with {len(msg.poses)} waypoints'
        self.get_logger().info(message)
        self.file_logger.info(message)
    
    def return_path_callback(self, msg):
        """Callback for return path messages"""
        self.return_path_count += 1
        self.latest_return_path = msg
        message = f'Received return path #{self.return_path_count} with {len(msg.poses)} waypoints'
        self.get_logger().info(message)
        self.file_logger.info(message)

    def home_trigger_callback(self, msg):
        """Callback for home trigger messages"""
        self.home_trigger_count += 1
        message = f'Received home trigger #{self.home_trigger_count}'
        self.get_logger().info(message)
        self.file_logger.info(message)

    def validate_paths(self):
        """Validate that the paths are being published correctly"""
        self.get_logger().info('Validating paths...')
        self.file_logger.info('Validating paths...')
        
        # Check if we've received any explore paths
        if self.explore_path_count > 0:
            message = f'✓ Explore paths received: {self.explore_path_count}'
            self.get_logger().info(message)
            self.file_logger.info(message)
            
            # Validate explore path structure
            if self.latest_explore_path:
                explore_poses = len(self.latest_explore_path.poses)
                message = f'  Latest explore path has {explore_poses} waypoints'
                self.get_logger().info(message)
                self.file_logger.info(message)
                
                # Basic validation: paths should have headers with correct frame
                if self.latest_explore_path.header.frame_id == 'map':
                    message = '  ✓ Explore path frame is correct (map)'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
                else:
                    message = f'  ✗ Explore path frame incorrect: {self.latest_explore_path.header.frame_id}'
                    self.get_logger().error(message)
                    self.file_logger.error(message)
                    return False
                
                # Check that waypoints have valid positions
                if explore_poses > 0:
                    first_pose = self.latest_explore_path.poses[0]
                    message = f'  ✓ First explore waypoint position: ({first_pose.pose.position.x:.2f}, {first_pose.pose.position.y:.2f})'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
                else:
                    message = '  ? Explore path has no waypoints yet'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
            else:
                message = '  ? No explore path data available'
                self.get_logger().info(message)
                self.file_logger.info(message)
        else:
            message = 'ℹ No explore paths received yet'
            self.get_logger().info(message)
            self.file_logger.info(message)
            
        # Check if we've received any return paths
        if self.return_path_count > 0:
            message = f'✓ Return paths received: {self.return_path_count}'
            self.get_logger().info(message)
            self.file_logger.info(message)
            
            # Validate return path structure
            if self.latest_return_path:
                return_poses = len(self.latest_return_path.poses)
                message = f'  Latest return path has {return_poses} waypoints'
                self.get_logger().info(message)
                self.file_logger.info(message)
                
                # Basic validation: paths should have headers with correct frame
                if self.latest_return_path.header.frame_id == 'map':
                    message = '  ✓ Return path frame is correct (map)'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
                else:
                    message = f'  ✗ Return path frame incorrect: {self.latest_return_path.header.frame_id}'
                    self.get_logger().error(message)
                    self.file_logger.error(message)
                    return False
                
                # Check that waypoints have valid positions
                if return_poses > 0:
                    first_pose = self.latest_return_path.poses[0]
                    message = f'  ✓ First return waypoint position: ({first_pose.pose.position.x:.2f}, {first_pose.pose.position.y:.2f})'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
                else:
                    message = '  ? Return path has no waypoints yet'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
            else:
                message = '  ? No return path data available'
                self.get_logger().info(message)
                self.file_logger.info(message)
        else:
            message = 'ℹ No return paths received yet'
            self.get_logger().info(message)
            self.file_logger.info(message)
            
        # Check home trigger
        if self.home_trigger_count > 0:
            message = f'✓ Home triggers received: {self.home_trigger_count}'
            self.get_logger().info(message)
            self.file_logger.info(message)
        else:
            message = 'ℹ No home triggers received yet'
            self.get_logger().info(message)
            self.file_logger.info(message)
            
        return True

    def startup_sync_callback(self, msg):
        """Callback for startup synchronization topic."""
        node_name = msg.data if hasattr(msg, 'data') else str(msg)
        
        if not node_name:
            self.get_logger().warn('Received empty node name')
            self.file_logger.warn('Received empty node name')
            return
            
        current_time = time.time()
        elapsed = current_time - self.start_time if self.start_time else 0.0
        
        if node_name in self.startup_sync_signals:
            message = f'[REPEAT] Received startup sync from {node_name} at {elapsed:.2f}s'
            self.get_logger().info(message)
            self.file_logger.info(message)
        else:
            self.startup_sync_signals[node_name] = current_time
            message = f'Received startup sync from {node_name} at {elapsed:.2f}s'
            self.get_logger().info(message)
            self.file_logger.info(message)
            
            # Check if all expected nodes are ready
            if set(self.startup_sync_signals.keys()) == self.expected_nodes:
                self.all_nodes_ready = True
                elapsed = time.time() - self.start_time
                message = f'=== ALL NODES READY! ==='
                self.get_logger().info(message)
                self.file_logger.info(message)
                message = f'Total time to startup sync: {elapsed:.2f} seconds'
                self.get_logger().info(message)
                self.file_logger.info(message)
                message = f'Received signals from: {sorted(self.startup_sync_signals.keys())}'
                self.get_logger().info(message)
                self.file_logger.info(message)
                
                # Print summary
                message = '--- Startup Sync Timing Summary ---'
                self.get_logger().info(message)
                self.file_logger.info(message)
                for node, timestamp in sorted(self.startup_sync_signals.items()):
                    message = f'{node}: {timestamp - self.start_time:.2f}s'
                    self.get_logger().info(message)
                    self.file_logger.info(message)
                
                # Publish trigger_start to trigger the challenge
                message = 'Publishing trigger_start to start challenge...'
                self.get_logger().info(message)
                self.file_logger.info(message)
                self.pub_trigger_start.publish(TRIGGER_START_INTERFACE())
                message = f'Published to {TRIGGER_START_TOPIC}'
                self.get_logger().info(message)
                self.file_logger.info(message)

    def run_test(self):
       """Run the runtime test until interrupted by keyboard signal"""
       self.get_logger().info('-' * 60)
       self.get_logger().info('PHASE 0: TEST INITIALIZATION')
       self.get_logger().info('-' * 60)
       self.get_logger().info('Starting runtime test - will run until interrupted by Ctrl+C...')
       self.file_logger.info('-' * 60)
       self.file_logger.info('PHASE 0: TEST INITIALIZATION')
       self.file_logger.info('-' * 60)
       self.file_logger.info('Starting runtime test - will run until interrupted by Ctrl+C...')
       
       # Wait for startup sync to complete before proceeding
       self.get_logger().info(f'Waiting for startup sync from nodes: {sorted(self.expected_nodes)}...')
       self.file_logger.info(f'Waiting for startup sync from nodes: {sorted(self.expected_nodes)}...')
       startup_sync_start = time.time()
       
       while not self.interrupted and not self.all_nodes_ready:
           rclpy.spin_once(self, timeout_sec=0.1)
           elapsed = time.time() - startup_sync_start
           if elapsed > TEST_SYNC_CHECK_TIMEOUT and not self.all_nodes_ready:
               message = 'Startup sync timeout - continuing anyway...'
               self.get_logger().warn(message)
               self.file_logger.warn(message)
               break
       
       if self.all_nodes_ready:
           message = f'Startup sync completed successfully in {time.time() - startup_sync_start:.2f} seconds'
           self.get_logger().info(message)
           self.file_logger.info(message)
       else:
           message = 'Startup sync did not complete within timeout'
           self.get_logger().error(message)
           self.file_logger.error(message)
       
       # Run indefinitely until interrupted (after startup sync is done)
       self.get_logger().info('-' * 60)
       self.get_logger().info('PHASE 4: CHALLENGE IN PROGRESS')
       self.get_logger().info('-' * 60)
       self.file_logger.info('-' * 60)
       self.file_logger.info('PHASE 4: CHALLENGE IN PROGRESS')
       self.file_logger.info('-' * 60)
       while not self.interrupted:
           rclpy.spin_once(self, timeout_sec=0.1)
       
       # Final validation when interrupted
       self.get_logger().info('-' * 60)
       self.get_logger().info('PHASE 5: TEST COMPLETED')
       self.get_logger().info('-' * 60)
       self.get_logger().info('Test interrupted by user. Performing final validation on current data...')
       self.file_logger.info('-' * 60)
       self.file_logger.info('PHASE 5: TEST COMPLETED')
       self.file_logger.info('-' * 60)
       self.file_logger.info('Test interrupted by user. Performing final validation on current data...')
       
       success = self.validate_paths()
       
       # Write summary to summary file
       test_duration = time.time() - self.start_time
       with open(self.summary_file_path, 'w') as f:
           f.write('=' * 60 + '\n')
           f.write('TEST SUMMARY\n')
           f.write('=' * 60 + '\n')
           f.write(f'Explore paths: {self.explore_path_count}\n')
           f.write(f'Return paths: {self.return_path_count}\n')
           f.write(f'Home triggers: {self.home_trigger_count}\n')
           f.write(f'Test duration: {test_duration:.2f} seconds\n')
           f.write(f'Test completed successfully: {success}\n')
           f.write(f'Log file: {self.log_file_path}\n')
           f.write('\n')
           f.write('Startup Sync Timing Summary:\n')
           for node, timestamp in sorted(self.startup_sync_signals.items()):
               f.write(f'  {node}: {timestamp - self.start_time:.2f}s\n')
       
       self.get_logger().info('')
       self.get_logger().info('=' * 60)
       self.get_logger().info('TEST SUMMARY')
       self.get_logger().info('=' * 60)
       self.get_logger().info(f'Explore paths: {self.explore_path_count}')
       self.get_logger().info(f'Return paths: {self.return_path_count}')
       self.get_logger().info(f'Home triggers: {self.home_trigger_count}')
       self.get_logger().info(f'Test duration: {test_duration:.2f} seconds')
       self.get_logger().info(f'Test completed successfully: {success}')
       self.get_logger().info(f'Summary file: {self.summary_file_path}')
       
       if success:
           self.get_logger().info('✓ Test completed successfully')
       else:
           self.get_logger().info('✗ Test had validation issues')
           
       return success

    def signal_handler(self, signum, frame):
        """Handle keyboard interrupt signal"""
        self.interrupted = True
        self.get_logger().info('Received interrupt signal. Stopping test...')
        self.file_logger.info('Received interrupt signal. Stopping test...')
        # Flush the log file
        for handler in self.file_logger.handlers:
            handler.flush()
        # Cancel the timer if it exists
        return True


def main():
    rclpy.init()
    
    # Create test node
    test_node = PathTracingRuntimeTest()
    
    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, test_node.signal_handler)
    
    try:
        # Run the test
        success = test_node.run_test()
        
        # Clean shutdown - close file logger handlers
        for handler in test_node.file_logger.handlers:
            handler.close()
        test_node.file_logger.handlers.clear()
        
        test_node.destroy_node()
        rclpy.shutdown()
        
        # Return exit code based on success
        return 0 if success else 1
        
    except Exception as e:
        test_node.get_logger().error(f'Unexpected error occurred: {e}')
        test_node.destroy_node()
        rclpy.shutdown()
        return 1


if __name__ == '__main__':
    sys.exit(main())
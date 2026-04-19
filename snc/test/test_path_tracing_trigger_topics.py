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
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('PATH TRACING RUNTIME TEST - INITIALIZING')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Subscribed to topics:')
        self.get_logger().info(f'  - {PATH_EXPLORE_TOPIC}')
        self.get_logger().info(f'  - {PATH_RETURN_TOPIC}')
        self.get_logger().info(f'  - {TRIGGER_HOME_TOPIC}')
        self.get_logger().info(f'  - {TRIGGER_START_TOPIC}')

    def explore_path_callback(self, msg):
        """Callback for explore path messages"""
        self.explore_path_count += 1
        self.latest_explore_path = msg
        self.get_logger().info(f'Received explore path #{self.explore_path_count} '
                              f'with {len(msg.poses)} waypoints')
    
    def return_path_callback(self, msg):
        """Callback for return path messages"""
        self.return_path_count += 1
        self.latest_return_path = msg
        self.get_logger().info(f'Received return path #{self.return_path_count} '
                              f'with {len(msg.poses)} waypoints')

    def home_trigger_callback(self, msg):
        """Callback for home trigger messages"""
        self.home_trigger_count += 1
        self.get_logger().info(f'Received home trigger #{self.home_trigger_count}')

    def validate_paths(self):
        """Validate that the paths are being published correctly"""
        self.get_logger().info('Validating paths...')
        
        # Check if we've received any explore paths
        if self.explore_path_count > 0:
            self.get_logger().info(f'✓ Explore paths received: {self.explore_path_count}')
            
            # Validate explore path structure
            if self.latest_explore_path:
                explore_poses = len(self.latest_explore_path.poses)
                self.get_logger().info(f'  Latest explore path has {explore_poses} waypoints')
                
                # Basic validation: paths should have headers with correct frame
                if self.latest_explore_path.header.frame_id == 'map':
                    self.get_logger().info('  ✓ Explore path frame is correct (map)')
                else:
                    self.get_logger().error(f'  ✗ Explore path frame incorrect: {self.latest_explore_path.header.frame_id}')
                    return False
                
                # Check that waypoints have valid positions
                if explore_poses > 0:
                    first_pose = self.latest_explore_path.poses[0]
                    self.get_logger().info(f'  ✓ First explore waypoint position: ({first_pose.pose.position.x:.2f}, {first_pose.pose.position.y:.2f})')
                else:
                    self.get_logger().info('  ? Explore path has no waypoints yet')
            else:
                self.get_logger().info('  ? No explore path data available')
        else:
            self.get_logger().info('ℹ No explore paths received yet')
            
        # Check if we've received any return paths
        if self.return_path_count > 0:
            self.get_logger().info(f'✓ Return paths received: {self.return_path_count}')
            
            # Validate return path structure
            if self.latest_return_path:
                return_poses = len(self.latest_return_path.poses)
                self.get_logger().info(f'  Latest return path has {return_poses} waypoints')
                
                # Basic validation: paths should have headers with correct frame
                if self.latest_return_path.header.frame_id == 'map':
                    self.get_logger().info('  ✓ Return path frame is correct (map)')
                else:
                    self.get_logger().error(f'  ✗ Return path frame incorrect: {self.latest_return_path.header.frame_id}')
                    return False
                
                # Check that waypoints have valid positions
                if return_poses > 0:
                    first_pose = self.latest_return_path.poses[0]
                    self.get_logger().info(f'  ✓ First return waypoint position: ({first_pose.pose.position.x:.2f}, {first_pose.pose.position.y:.2f})')
                else:
                    self.get_logger().info('  ? Return path has no waypoints yet')
            else:
                self.get_logger().info('  ? No return path data available')
        else:
            self.get_logger().info('ℹ No return paths received yet')
            
        # Check home trigger
        if self.home_trigger_count > 0:
            self.get_logger().info(f'✓ Home triggers received: {self.home_trigger_count}')
        else:
            self.get_logger().info('ℹ No home triggers received yet')
            
        return True

    def startup_sync_callback(self, msg):
        """Callback for startup synchronization topic."""
        node_name = msg.data if hasattr(msg, 'data') else str(msg)
        
        if not node_name:
            self.get_logger().warn('Received empty node name')
            return
            
        current_time = time.time()
        elapsed = current_time - self.start_time if self.start_time else 0.0
        
        if node_name in self.startup_sync_signals:
            self.get_logger().info(f'[REPEAT] Received startup sync from {node_name} at {elapsed:.2f}s')
        else:
            self.startup_sync_signals[node_name] = current_time
            self.get_logger().info(f'Received startup sync from {node_name} at {elapsed:.2f}s')
            
            # Check if all expected nodes are ready
            if set(self.startup_sync_signals.keys()) == self.expected_nodes:
                self.all_nodes_ready = True
                elapsed = time.time() - self.start_time
                self.get_logger().info(f'=== ALL NODES READY! ===')
                self.get_logger().info(f'Total time to startup sync: {elapsed:.2f} seconds')
                self.get_logger().info(f'Received signals from: {sorted(self.startup_sync_signals.keys())}')
                
                # Print summary
                self.get_logger().info('--- Startup Sync Timing Summary ---')
                for node, timestamp in sorted(self.startup_sync_signals.items()):
                    self.get_logger().info(f'{node}: {timestamp - self.start_time:.2f}s')
                
                # Publish trigger_start to trigger the challenge
                self.get_logger().info('Publishing trigger_start to start challenge...')
                self.pub_trigger_start.publish(TRIGGER_START_INTERFACE())
                self.get_logger().info(f'Published to {TRIGGER_START_TOPIC}')

    def run_test(self):
       """Run the runtime test until interrupted by keyboard signal"""
       self.get_logger().info('-' * 60)
       self.get_logger().info('PHASE 0: TEST INITIALIZATION')
       self.get_logger().info('-' * 60)
       self.get_logger().info('Starting runtime test - will run until interrupted by Ctrl+C...')
       
       # Wait for startup sync to complete before proceeding
       self.get_logger().info(f'Waiting for startup sync from nodes: {sorted(self.expected_nodes)}...')
       startup_sync_start = time.time()
       
       while not self.interrupted and not self.all_nodes_ready:
           rclpy.spin_once(self, timeout_sec=0.1)
           elapsed = time.time() - startup_sync_start
           if elapsed > 30.0 and not self.all_nodes_ready:
               self.get_logger().warn('Startup sync timeout - continuing anyway...')
               break
       
       if self.all_nodes_ready:
           self.get_logger().info(f'Startup sync completed successfully in {time.time() - startup_sync_start:.2f} seconds')
       else:
           self.get_logger().error('Startup sync did not complete within timeout')
       
       # Run indefinitely until interrupted (after startup sync is done)
       self.get_logger().info('-' * 60)
       self.get_logger().info('PHASE 4: CHALLENGE IN PROGRESS')
       self.get_logger().info('-' * 60)
       while not self.interrupted:
           rclpy.spin_once(self, timeout_sec=0.1)
       
       # Final validation when interrupted
       self.get_logger().info('-' * 60)
       self.get_logger().info('PHASE 5: TEST COMPLETED')
       self.get_logger().info('-' * 60)
       self.get_logger().info('Test interrupted by user. Performing final validation on current data...')
       
       success = self.validate_paths()
       
       # Summary
       self.get_logger().info('')
       self.get_logger().info('=' * 60)
       self.get_logger().info('TEST SUMMARY')
       self.get_logger().info('=' * 60)
       self.get_logger().info(f'Explore paths: {self.explore_path_count}')
       self.get_logger().info(f'Return paths: {self.return_path_count}')
       self.get_logger().info(f'Home triggers: {self.home_trigger_count}')
       self.get_logger().info('Test duration: indefinite (until interrupted)')
       
       if success:
           self.get_logger().info('✓ Test completed successfully')
       else:
           self.get_logger().info('✗ Test had validation issues')
           
       return success

    def signal_handler(self, signum, frame):
        """Handle keyboard interrupt signal"""
        self.interrupted = True
        self.get_logger().info('Received interrupt signal. Stopping test...')
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
        
        # Clean shutdown
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
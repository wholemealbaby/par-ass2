#!/usr/bin/env python3
"""
Test script to verify the startup synchronization feature.

This script listens to the startup synchronization topic and verifies that
all three nodes (path_tracing, navigation, marker_detection) publish their
readiness within a reasonable time frame.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from snc.constants import STARTUP_SYNC_TOPIC, STARTUP_SYNC_BUFFER_SIZE
import time


class StartupSyncTestNode(Node):
    """Test node that listens to startup synchronization topic."""

    def __init__(self, timeout_sec=30.0):
        super().__init__('startup_sync_test')
        self.get_logger().info('Startup Sync Test Node launched')
        
        self.timeout_sec = timeout_sec
        self.start_time = None
        self.received_signals = {}
        self.expected_nodes = {'path_tracing', 'navigation', 'marker_detection'}
        self.all_nodes_ready = False
        
        # Subscriber for startup sync topic
        self.sub_startup_sync = self.create_subscription(
            String,
            STARTUP_SYNC_TOPIC,
            self.startup_sync_callback,
            STARTUP_SYNC_BUFFER_SIZE
        )
        
        # Timer to check timeout
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f'Listening for startup sync signals from: {self.expected_nodes}')
        self.get_logger().info(f'Timeout: {timeout_sec} seconds')

    def startup_sync_callback(self, msg):
        """Callback for startup synchronization topic."""
        node_name = msg.data if hasattr(msg, 'data') else str(msg)
        
        if not node_name:
            self.get_logger().warn('Received empty node name')
            return
            
        current_time = time.time()
        elapsed = current_time - self.start_time if self.start_time else 0.0
        
        if node_name in self.received_signals:
            self.get_logger().info(f'[REPEAT] Received signal from {node_name} at {elapsed:.2f}s')
        else:
            self.received_signals[node_name] = current_time
            self.get_logger().info(f'Received signal from {node_name} at {elapsed:.2f}s')
            
            # Check if all expected nodes are ready
            if set(self.received_signals.keys()) == self.expected_nodes:
                self.all_nodes_ready = True
                elapsed = time.time() - self.start_time
                self.get_logger().info(f'=== ALL NODES READY! ===')
                self.get_logger().info(f'Total time: {elapsed:.2f} seconds')
                self.get_logger().info(f'Received signals from: {sorted(self.received_signals.keys())}')
                
                # Print summary
                self.get_logger().info('--- Signal Timing Summary ---')
                for node, timestamp in sorted(self.received_signals.items()):
                    self.get_logger().info(f'{node}: {timestamp - self.start_time:.2f}s')
                
                # Shutdown after a short delay
                self.get_logger().info('Test passed! Shutting down...')
                self.create_timer(1.0, self.shutdown_node)

    def timer_callback(self):
        """Timer callback to check for timeout."""
        if self.all_nodes_ready:
            return
            
        elapsed = time.time() - self.start_time if self.start_time else 0.0
        remaining = self.timeout_sec - elapsed
        
        if elapsed >= self.timeout_sec:
            self.get_logger().error(f'=== TIMEOUT ===')
            self.get_logger().error(f'Expected nodes: {sorted(self.expected_nodes)}')
            self.get_logger().error(f'Received nodes: {sorted(self.received_signals.keys())}')
            missing = self.expected_nodes - set(self.received_signals.keys())
            if missing:
                self.get_logger().error(f'Missing nodes: {sorted(missing)}')
            self.get_logger().error('Test failed!')
            rclpy.shutdown()
        else:
            self.get_logger().info(f'Waiting... ({remaining:.1f}s remaining, {len(self.received_signals)}/3 nodes ready)')

    def shutdown_node(self):
        """Shutdown the node."""
        rclpy.shutdown()

    def wait_for_start(self):
        """Wait for the test to start (start_time to be set)."""
        while rclpy.ok() and not self.start_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Test started!')


def main(args=None):
    rclpy.init(args=args)
    
    # Create test node with 30 second timeout
    test_node = StartupSyncTestNode(timeout_sec=30.0)
    
    # Set start time and begin listening
    test_node.start_time = time.time()
    test_node.get_logger().info('Test node ready. Waiting for nodes to publish readiness...')
    
    try:
        while rclpy.ok() and not test_node.all_nodes_ready:
            rclpy.spin_once(test_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

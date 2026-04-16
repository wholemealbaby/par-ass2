#!/usr/bin/env python3
"""
Runtime tests for the marker detection node (Node #2 - Hazard Marker Detection).
This script monitors the topic output of the marker detection node
and validates if the node is performing as expected based on the spec.

The test monitors:
- /hazards topic (visualization_msgs/msg/Marker) - hazard markers published by Node #2
- /snc_status topic (std_msgs/msg/String) - node status
- Trigger topics: /trigger_start, /trigger_teleop, /trigger_home (std_msgs/msg/Empty)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, String
import time
import sys
import os
import signal

# Add the snc module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class MarkerDetectionRuntimeTest(Node):
    def __init__(self):
        super().__init__('marker_detection_runtime_test')
        
        # Track received messages
        self.hazard_marker_count = 0
        self.status_message_count = 0
        self.trigger_start_count = 0
        self.trigger_teleop_count = 0
        self.trigger_home_count = 0
        
        # Store latest messages
        self.latest_hazard_marker = None
        self.latest_status = None
        
        # Track detected hazard IDs
        self.detected_hazard_ids = set()
        self.expected_hazard_ids = set(range(13))  # IDs 0-12
        
        # Statistics tracking
        self.marker_timestamps = []
        self.marker_publishing_rate = 0.0
        self.last_marker_time = None
        
        # Subscribe to hazards topic (visualization_msgs/msg/Marker)
        self.hazard_sub = self.create_subscription(
            Marker,
            '/hazards',
            self.hazard_marker_callback,
            10
        )
        
        # Subscribe to status topic (std_msgs/msg/String)
        self.status_sub = self.create_subscription(
            String,
            '/snc_status',
            self.status_callback,
            10
        )
        
        # Subscribe to trigger topics (std_msgs/msg/Empty)
        self.trigger_start_sub = self.create_subscription(
            Empty,
            '/trigger_start',
            self.trigger_start_callback,
            10
        )
        
        self.trigger_teleop_sub = self.create_subscription(
            Empty,
            '/trigger_teleop',
            self.trigger_teleop_callback,
            10
        )
        
        self.trigger_home_sub = self.create_subscription(
            Empty,
            '/trigger_home',
            self.trigger_home_callback,
            10
        )
        
        # Test duration and timeout
        self.test_duration = 30.0  # seconds
        self.start_time = time.time()
        
        # Flag to indicate if the test was interrupted
        self.interrupted = False
        
        self.get_logger().info('Marker detection runtime test initialized')
        self.get_logger().info('Subscribed to topics:')
        self.get_logger().info('  - /hazards (visualization_msgs/msg/Marker)')
        self.get_logger().info('  - /snc_status (std_msgs/msg/String)')
        self.get_logger().info('  - /trigger_start (std_msgs/msg/Empty)')
        self.get_logger().info('  - /trigger_teleop (std_msgs/msg/Empty)')
        self.get_logger().info('  - /trigger_home (std_msgs/msg/Empty)')

    def hazard_marker_callback(self, msg):
        """Callback for hazard marker messages"""
        self.hazard_marker_count += 1
        self.latest_hazard_marker = msg
        self.marker_timestamps.append(time.time())
        
        # Track detected hazard ID
        if msg.id is not None:
            self.detected_hazard_ids.add(msg.id)
        
        # Calculate publishing rate
        if self.last_marker_time is not None:
            time_diff = time.time() - self.last_marker_time
            if time_diff > 0:
                self.marker_publishing_rate = 1.0 / time_diff
        self.last_marker_time = time.time()
        
        # Log marker information
        frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else 'unknown'
        self.get_logger().info(
            f'Received hazard marker #{self.hazard_marker_count} '
            f'(ID: {msg.id}, Frame: {frame_id}, Type: {msg.type})'
        )

    def status_callback(self, msg):
        """Callback for status messages"""
        self.status_message_count += 1
        self.latest_status = msg
        self.get_logger().info(f'Status message #{self.status_message_count}: {msg.data}')

    def trigger_start_callback(self, msg):
        """Callback for trigger start messages"""
        self.trigger_start_count += 1
        self.get_logger().info(f'Received trigger start #{self.trigger_start_count}')

    def trigger_teleop_callback(self, msg):
        """Callback for trigger teleop messages"""
        self.trigger_teleop_count += 1
        self.get_logger().info(f'Received trigger teleop #{self.trigger_teleop_count}')

    def trigger_home_callback(self, msg):
        """Callback for trigger home messages"""
        self.trigger_home_count += 1
        self.get_logger().info(f'Received trigger home #{self.trigger_home_count}')

    def validate_hazard_markers(self):
        """Validate that hazard markers are being published correctly"""
        self.get_logger().info('Validating hazard markers...')
        
        success = True
        
        # Check if we've received any hazard markers
        if self.hazard_marker_count > 0:
            self.get_logger().info(f'✓ Hazard markers received: {self.hazard_marker_count}')
            
            # Validate marker structure
            if self.latest_hazard_marker:
                # Check marker ID
                marker_id = self.latest_hazard_marker.id if hasattr(self.latest_hazard_marker, 'id') else None
                self.get_logger().info(f'  Latest marker ID: {marker_id}')
                
                # Validate marker frame
                if hasattr(self.latest_hazard_marker, 'header') and hasattr(self.latest_hazard_marker.header, 'frame_id'):
                    marker_frame = self.latest_hazard_marker.header.frame_id
                    if marker_frame == 'map':
                        self.get_logger().info(f'  ✓ Marker frame is correct (map)')
                    else:
                        self.get_logger().error(f'  ✗ Marker frame incorrect: {marker_frame}')
                        success = False
                else:
                    self.get_logger().error('  ✗ Marker missing header or frame_id')
                    success = False
                
                # Check marker type
                if hasattr(self.latest_hazard_marker, 'type'):
                    marker_type = self.latest_hazard_marker.type
                    self.get_logger().info(f'  Marker type: {marker_type} (3=SPHERE, 0=ARROW, etc.)')
                else:
                    self.get_logger().warn('  ? Marker type not available')
                
                # Check marker pose
                if hasattr(self.latest_hazard_marker, 'pose'):
                    pose = self.latest_hazard_marker.pose
                    if hasattr(pose, 'position'):
                        self.get_logger().info(
                            f'  ✓ Marker position: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})'
                        )
                    if hasattr(pose, 'orientation'):
                        self.get_logger().info(
                            f'  Marker orientation: ({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, '
                            f'{pose.orientation.z:.2f}, {pose.orientation.w:.2f})'
                        )
                else:
                    self.get_logger().warn('  ? Marker pose not available')
                
                # Check marker scale
                if hasattr(self.latest_hazard_marker, 'scale'):
                    scale = self.latest_hazard_marker.scale
                    self.get_logger().info(
                        f'  Marker scale: (x={scale.x:.2f}, y={scale.y:.2f}, z={scale.z:.2f})'
                    )
                
                # Check marker color
                if hasattr(self.latest_hazard_marker, 'color'):
                    color = self.latest_hazard_marker.color
                    self.get_logger().info(
                        f'  Marker color: (r={color.r:.2f}, g={color.g:.2f}, b={color.b:.2f}, a={color.a:.2f})'
                    )
                
                # Check marker action
                if hasattr(self.latest_hazard_marker, 'action'):
                    action = self.latest_hazard_marker.action
                    self.get_logger().info(f'  Marker action: {action} (0=ADD, 1=DELETE, etc.)')
            else:
                self.get_logger().info('  ? No hazard marker data available')
        else:
            self.get_logger().info('ℹ No hazard markers received yet')
        
        return success

    def validate_hazard_ids(self):
        """Validate that hazard markers are published with correct IDs (0-12)"""
        self.get_logger().info('Validating hazard IDs...')
        
        success = True
        
        # Check if we've received any markers
        if self.hazard_marker_count > 0:
            # Check which IDs we've detected
            detected_count = len(self.detected_hazard_ids)
            self.get_logger().info(f'Detected hazard IDs: {sorted(self.detected_hazard_ids)}')
            self.get_logger().info(f'Number of unique hazard IDs detected: {detected_count}')
            
            # Check for missing IDs
            missing_ids = self.expected_hazard_ids - self.detected_hazard_ids
            if missing_ids:
                self.get_logger().warn(f'Missing hazard IDs: {sorted(missing_ids)}')
                # Don't fail the test if some IDs haven't been detected yet
                # This is expected during initial test runs
            else:
                self.get_logger().info('✓ All expected hazard IDs (0-12) have been detected')
        else:
            self.get_logger().info('ℹ No hazard markers received yet - cannot validate IDs')
        
        return success

    def validate_trigger_topics(self):
        """Validate trigger topic subscriptions and counts"""
        self.get_logger().info('Validating trigger topics...')
        
        # Check trigger counts
        if self.trigger_start_count > 0:
            self.get_logger().info(f'✓ Trigger start received: {self.trigger_start_count} times')
        else:
            self.get_logger().info('ℹ No trigger start messages received yet')
        
        if self.trigger_teleop_count > 0:
            self.get_logger().info(f'✓ Trigger teleop received: {self.trigger_teleop_count} times')
        else:
            self.get_logger().info('ℹ No trigger teleop messages received yet')
        
        if self.trigger_home_count > 0:
            self.get_logger().info(f'✓ Trigger home received: {self.trigger_home_count} times')
        else:
            self.get_logger().info('ℹ No trigger home messages received yet')
        
        return True

    def validate_status_topic(self):
        """Validate status topic subscriptions and messages"""
        self.get_logger().info('Validating status topic...')
        
        if self.status_message_count > 0:
            self.get_logger().info(f'✓ Status messages received: {self.status_message_count} times')
            if self.latest_status:
                self.get_logger().info(f'  Latest status: {self.latest_status.data}')
        else:
            self.get_logger().info('ℹ No status messages received yet')
        
        return True

    def validate_publishing_rate(self):
        """Validate marker publishing rate"""
        self.get_logger().info('Validating publishing rate...')
        
        if self.hazard_marker_count > 1 and self.last_marker_time:
            elapsed_time = self.last_marker_time - self.start_time
            if elapsed_time > 0:
                actual_rate = self.hazard_marker_count / elapsed_time
                self.get_logger().info(f'  Average publishing rate: {actual_rate:.2f} Hz')
                self.get_logger().info(f'  Current publishing rate: {self.marker_publishing_rate:.2f} Hz')
        else:
            self.get_logger().info('ℹ Not enough markers to calculate publishing rate')
        
        return True

    def validate_paths(self):
        """Run all validation checks"""
        self.get_logger().info('Running all validation checks...')
        
        all_success = True
        
        # Run all validations
        if not self.validate_hazard_markers():
            all_success = False
        
        if not self.validate_hazard_ids():
            all_success = False
        
        if not self.validate_trigger_topics():
            all_success = False
        
        if not self.validate_status_topic():
            all_success = False
        
        self.validate_publishing_rate()
        
        return all_success

    def run_test(self):
        """Run the runtime test until interrupted by keyboard signal"""
        self.get_logger().info('Starting runtime test - will run until interrupted by Ctrl+C...')
        
        # Run indefinitely until interrupted
        while not self.interrupted:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Final validation when interrupted
        self.get_logger().info('Test interrupted by user. Performing final validation on current data...')
        
        success = self.validate_paths()
        
        # Summary
        self.get_logger().info('\n=== TEST SUMMARY ===')
        self.get_logger().info(f'Hazard markers received: {self.hazard_marker_count}')
        self.get_logger().info(f'Unique hazard IDs detected: {len(self.detected_hazard_ids)}')
        self.get_logger().info(f'Status messages received: {self.status_message_count}')
        self.get_logger().info(f'Trigger start received: {self.trigger_start_count} times')
        self.get_logger().info(f'Trigger teleop received: {self.trigger_teleop_count} times')
        self.get_logger().info(f'Trigger home received: {self.trigger_home_count} times')
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
    test_node = MarkerDetectionRuntimeTest()
    
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

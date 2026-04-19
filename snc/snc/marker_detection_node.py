#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from snc.constants import (
    START_CHALLENGE_INTERFACE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_BUFFER_SIZE,
    OBJECTS_TOPIC,
    OBJECTS_BUFFER_SIZE,
    OBJECTS_INTERFACE,
    STARTUP_SYNC_TOPIC,
    STARTUP_SYNC_INTERFACE,
    STARTUP_SYNC_BUFFER_SIZE,
    STARTUP_SYNC_QOS,
)
from std_msgs.msg import Empty, String
from snc.hazard import HazardManager
from snc.object import ObjectHandler

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        self.get_logger().info('Marker detection node launched')
        
        # Flag to ensure start marker is only once
        self.start_marker_detected = False

        # Handler for decoding find object 2D data into objects
        self.object_handler = ObjectHandler()
        # Handler for keeping track of unique hazards
        # and transforming their positions into the map frame
        self.hazard_manager = HazardManager()


        self.sub_objects = self.create_subscription(
            OBJECTS_INTERFACE,
            OBJECTS_TOPIC,
            self.objects_callback,
            OBJECTS_BUFFER_SIZE,
        )
        
        # Create publisher to /snc_start topic
        self.pub_start_challenge = self.create_publisher(
            START_CHALLENGE_INTERFACE,
            START_CHALLENGE_TOPIC,
            START_CHALLENGE_BUFFER_SIZE,
        )

        # Startup synchronization publisher - publishes node readiness
        self.pub_startup_sync = self.create_publisher(
            STARTUP_SYNC_INTERFACE,
            STARTUP_SYNC_TOPIC,
            STARTUP_SYNC_QOS
        )

        # Startup synchronization subscriber - waits for all nodes to be ready
        self.sub_startup_sync = self.create_subscription(
            STARTUP_SYNC_INTERFACE,
            STARTUP_SYNC_TOPIC,
            self.startup_sync_callback,
            STARTUP_SYNC_QOS
        )

        # Track which nodes have published readiness
        self.nodes_ready = set()
        self.node_name = 'marker_detection'
        self.all_nodes_ready = False

    def trigger_start(self) -> None:
        """Set the internal flag (self.start_marker_detected) to True
        and publish to the start challenge topic."""
        self.start_marker_detected = True
        self.pub_start_challenge.publish(Empty())
        self.get_logger().info('Start marker detected! Published to /snc_start.')

    def objects_callback(self, msg):
        """Callback method to handle published object information."""
        self.get_logger().debug('Received objects message, processing...')
        # Use handler to decode object data
        self.object_handler.add_objects_from_message(msg)
        if not self.start_marker_detected and self.object_handler.start_marker_detected():
            self.trigger_start()
        self.hazard_manager.update_from_objects(self.object_handler.objects)
        hazards = self.hazard_manager.get_unique_hazards()
        self.get_logger().info(f"Unique hazards detected: {hazards}")
    
    def startup_sync_callback(self, msg):
        """Callback for startup synchronization topic. Tracks which nodes have published readiness."""
        if self.all_nodes_ready:
            return
        
        # Extract node name from the message data
        node_name = msg.data if hasattr(msg, 'data') else str(msg)
        if node_name and node_name != self.node_name:
            self.nodes_ready.add(node_name)
            self.get_logger().info(f'Received ready signal from: {node_name}. Ready nodes: {len(self.nodes_ready) + 1}/3')
            
            # Check if all expected nodes are ready (3 nodes: path_tracing, navigation, marker_detection)
            if len(self.nodes_ready) >= 2:  # +1 for self
                self.all_nodes_ready = True
                self.get_logger().info('All nodes are ready! Starting startup synchronization complete.')
    
    def wait_for_all_nodes_ready(self):
        """Wait for all nodes to publish their readiness on the startup sync topic."""
        self.get_logger().info('Waiting for all nodes to be ready...')
        
        # Publish this node's readiness
        self.pub_startup_sync.publish(String(data=self.node_name))
        
        while rclpy.ok() and not self.all_nodes_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('All nodes ready, proceeding with initialization.')

    def timer_callback(self):
        pass
        

def main():
    rclpy.init()
    node = MarkerDetectionNode()

    # Wait for all nodes to be ready before starting
    node.wait_for_all_nodes_ready()
    
    try:
       rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

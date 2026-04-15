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
)
from std_msgs.msg import Empty
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

    def trigger_start(self) -> None:
        """Set the internal flag (self.start_marker_detected) to True
        and publish to the start challenge topic."""
        self.start_marker_detected = True
        self.pub_start_challenge.publish(Empty())
        self.get_logger().info('Start marker detected! Published to /snc_start.')

    def objects_callback(self, msg):
        """Callback method to handle published object information."""
        # Use handler to decode object data
        self.object_handler.add_objects_from_message(msg)
        if not self.start_marker_detected and self.object_handler.start_marker_detected:
            self.trigger_start()
        self.hazard_manager.update_from_objects(self.object_handler.objects)
        
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

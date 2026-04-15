#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from constants import (
    START_CHALLENGE_INTERFACE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_BUFFER_SIZE,
    OBJECTS_TOPIC,
    OBJECTS_BUFFER_SIZE,
    OBJECTS_INTERFACE,
)
from std_msgs.msg import Empty
from hazard import Hazards

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')
        self.get_logger().info('Marker detection node launched')
        
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

    def objects_callback(self, msg):
        """Callback method to handle published object information."""
        # Decode into hazard objects
        self.hazards = Hazards(msg)
        self.get_logger().info(f"Received {len(self.hazards.hazards)}"
        f" hazards: {"\n".join(h.name for h in self.hazards.hazards)}")
        for h in self.hazards.hazards:
            if h.name == "Start":
                self.get_logger().info("Start challenge marker detected! Publishing trigger...")
                self.pub_start_challenge.publish(Empty())
                return


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

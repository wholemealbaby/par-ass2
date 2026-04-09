#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
from constants import (
    PATH_EXPLORE_BUFFER_SIZE,
    PATH_RETURN_BUFFER_SIZE,
    HOME_TRIGGER_BUFFER_SIZE,
    HOME_TRIGGER_TOPIC,
    PATH_EXPLORE_TOPIC,
    PATH_RETURN_TOPIC
)
import math
from geometry_msgs.msg import Transform
import tf_transformations 

class PathTracingNode(Node):
    def __init__(self):
        super().__init__('path_tracing_node')
        self.get_logger().info('Path tracing node launched')
        
        # Load parameters
        self.pose_sample_interval_s = self.get_parameter('pose_sample_interval_s').get_parameter_value().double_value
        self.waypoint_spacing_min = self.get_parameter('waypoint_spacing_min').get_parameter_value().double_value

        # Timer to sample the robot's pose at regular intervals
        self.sample_pose_timer = self.create_timer(self.pose_sample_interval_s, self.sample_pose_callback)
        self.counter = 0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.breadcrumbs = []  # List to store the breadcrumb waypoints

        # Return home trigger
        self.sub_home_trigger = self.create_subscription(
            Empty,
            HOME_TRIGGER_TOPIC,
            self.home_trigger_callback,
            HOME_TRIGGER_BUFFER_SIZE
        )

        # Path exploration subscription for waypoints
        self.sub_path_explorer = self.create_subscription(
            Path,
            PATH_EXPLORE_TOPIC,
            self.path_explorer_callback,
            PATH_EXPLORE_BUFFER_SIZE
        )

        # Publisher for return waypoints
        self.pub_path_return = self.create_publisher(
            Path,
            PATH_RETURN_TOPIC,
            PATH_RETURN_BUFFER_SIZE
        )

    def sample_pose_callback(self):
        """Timer callback to periodically sample the robot's pose and update the path tracing
        """
        # check transform is possible
        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            # Log intermittently to avoid spamming the console
            self.get_logger().warn("Waiting for TF to become available...", throttle_duration_sec=10.0)
            return

        try:
            # Get current position
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_x = t.transform.translation.x
            current_y = t.transform.translation.y

            # Check if we have moved enough to care
            if self.last_recorded_pose is not None:
                dist = math.sqrt((current_x - self.last_recorded_pose[0])**2 + 
                                (current_y - self.last_recorded_pose[1])**2)
                
                if dist < self.distance_threshold:
                    return # Skip recording this time

            # Save the waypoint
            self.breadcrumbs.append(t)
            self.last_recorded_pose = (current_x, current_y)
            self.get_logger().info(f"Stored waypoint {len(self.breadcrumbs)}")

        except TransformException as e:
            self.get_logger().warn(f"Failed to get robot pose despite TF being available: {e}")


    def get_yaw_from_transform(self, t: Transform):
        """
        Extracts the yaw (Z-axis rotation) from a geometry_msgs/(Transform.
        """
        # Extract quaternion components 
        quaternion = (
            t.rotation.x,
            t.rotation.y,
            t.rotation.z,
            t.rotation.w
        )

        # Convert quaternion to roll, pitch, yaw
        # returns a list
        euler = tf_transformations.euler_from_quaternion(quaternion)

        # Return the yaw
        return euler[2]
    
    def home_trigger_callback(self, msg):
        self.get_logger().info('Home trigger received, starting path tracing')
    
    def wait_for_robot_pose(self):
        self.get_logger().info("Waiting for map to base_link transform...")
        
        # Block until the transform is available or 5 seconds pass
        while not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=5.0)):
            self.get_logger().info("Still waiting for TF...")
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Transform found! Starting path tracing.")

def main():
    rclpy.init()
    node = PathTracingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
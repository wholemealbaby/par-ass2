#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, TransformException
from snc.constants import (
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

SAMPLE_FAILED = 0
SAMPLE_SKIPPED = 2

class PathTracingNode(Node):
    def __init__(self):
        super().__init__('path_tracing_node')
        self.get_logger().info('Path tracing node launched')
        
        # Load parameters
        self.pose_sample_interval_s = self.get_parameter('pose_sample_interval_s').get_parameter_value().double_value
        self.waypoint_spacing_min = self.get_parameter('waypoint_spacing_min').get_parameter_value().double_value
        self.waypoint_rotation_min = self.get_parameter('waypoint_rotation_min').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.explore_breadcrumbs = []  # List to store the breadcrumb waypoints for the explore path
        self.return_breadcrumbs = []   # List to store the breadcrumb waypoints for the return path
        self.last_recorded_pose = None
        self.last_recorded_yaw = None
        self.return_triggered = False # Flag to indicate if return home has been triggered, stops pose sampling when true

        # Return home trigger
        self.sub_home_trigger = self.create_subscription(
            Empty,
            HOME_TRIGGER_TOPIC,
            self.home_trigger_callback,
            HOME_TRIGGER_BUFFER_SIZE
        )
        # Publisher for explore waypoints
        self.pub_path_explore = self.create_publisher(
            Path,
            PATH_EXPLORE_TOPIC,
            PATH_EXPLORE_BUFFER_SIZE
        )
        # Publisher for return waypoints
        self.pub_path_return = self.create_publisher(
            Path,
            PATH_RETURN_TOPIC,
            PATH_RETURN_BUFFER_SIZE
        )

        self.wait_for_robot_pose()

        # Timer to sample the robot's pose at regular intervals
        self.sample_pose_timer = self.create_timer(self.pose_sample_interval_s, self.sample_pose_callback)
    
    def check_base_link_map_transform_possible(self):
        """Checks if the transform between base_link and map is possible, which is required for path tracing to function. Logs intermittently if not available.
        """
        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            # Log intermittently to avoid spamming the console
            self.get_logger().warn("Waiting for TF to become available...", throttle_duration_sec=10.0)
            return False
        return True

    def sample_pose_callback(self):
        """Timer callback to periodically sample the robot's pose and update the path tracing. Skips if return home has been triggered.
        """
        # Check that the transform is possible
        if not self.check_base_link_map_transform_possible():
            return

        pose = self.get_robot_pose_in_map_frame()
        if pose == SAMPLE_SKIPPED or pose == SAMPLE_FAILED:
            return
        self.last_recorded_pos = (current_x, current_y)
        self.last_recorded_yaw = self.get_yaw_from_transform(t)
        # Save the waypoint to the appropriate breadcrumb list and publish the path
        if self.return_triggered:
            self.explore_breadcrumbs.append(pose)
            self.get_logger().info(f"Stored explore waypoint {len(self.explore_breadcrumbs)}")
            self.pub_path_explore.publish(Path(header=pose.header, poses=self.explore_breadcrumbs))
        else:
            self.return_breadcrumbs.append(pose)
            self.get_logger().info(f"Stored return waypoint {len(self.return_breadcrumbs)}")
            self.pub_path_return.publish(Path(header=pose.header, poses=self.return_breadcrumbs))

    def get_robot_pose_in_map_frame(self):
        try:
            # Get current position
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Failed to get robot pose despite TF being available: {e}")
            return SAMPLE_FAILED
        
        current_x = t.transform.translation.x
        current_y = t.transform.translation.y

        # Check if waypoint we minimums are satisfied before storing
        if self.last_recorded_pose is not None and self.last_recorded_yaw is not None:
            delta_dist = math.sqrt((current_x - self.last_recorded_pose[0])**2 + 
                            (current_y - self.last_recorded_pose[1])**2)
            delta_yaw = abs(self.get_yaw_from_transform(t) - self.last_recorded_yaw)
            
            # Skip if waypoint minimums aren't met
            if delta_dist < self.waypoint_spacing_min and delta_yaw < self.waypoint_rotation_min:
                return SAMPLE_SKIPPED

        # Convert to pose stamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = current_x
        pose.pose.position.y = current_y
        pose.pose.position.z = 0.0
        pose.pose.orientation = t.transform.rotation

        return pose


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
        self.get_logger().info("Pre flight check: Waiting for (robot pose) map to base_link transform...")
        
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
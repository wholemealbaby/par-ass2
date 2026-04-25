#!/usr/bin/env python3

from unittest.mock import Mock

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer, TransformException
from nav2_simple_commander.robot_navigator import BasicNavigator

from snc.constants import (
    GO_HOME_TOPIC,
    GO_HOME_INTERFACE,
    GO_HOME_BUFFER_SIZE,
    PATH_EXPLORE_TOPIC,
    PATH_EXPLORE_BUFFER_SIZE,
    PATH_EXPLORE_INTERFACE,
    PATH_RETURN_TOPIC,
    PATH_RETURN_BUFFER_SIZE,
    PATH_RETURN_INTERFACE,
    TRIGGER_START_TOPIC,
    TRIGGER_START_BUFFER_SIZE,
    TRIGGER_START_INTERFACE,
    TRIGGER_TELEOP_TOPIC,
    TRIGGER_TELEOP_BUFFER_SIZE,
    TRIGGER_TELEOP_INTERFACE,
    START_CHALLENGE_TOPIC,
    START_CHALLENGE_INTERFACE,
    START_CHALLENGE_BUFFER_SIZE,
    TRIGGER_HOME_TOPIC, TRIGGER_HOME_BUFFER_SIZE, TRIGGER_HOME_INTERFACE,
    SNC_STATUS_TOPIC, SNC_STATUS_INTERFACE, SNC_STATUS_BUFFER_SIZE,
    TRIGGER_QOS,
)
import math

from snc_interfaces.srv import ExplorationControl


# Import core functions for easier testing
from snc.path_tracing_core import (
    should_record_waypoint,
    construct_pose_stamped,
    euler_from_quaternion,
    SAMPLE_FAILED,
    SAMPLE_SKIPPED,
    calculate_return_trajectory
)


class PathTracingNode(Node):
    def __init__(self, nav, params=None):
        """
        Initialize the PathTracingNode.
        
        Args:
            nav: The BasicNavigator instance to use for navigation
            params: Optional dictionary of parameters to override defaults
        """
        super().__init__('path_tracing_node')
        self.get_logger().info('=' * 60)
        self.get_logger().info('PATH TRACING NODE - INITIALIZING')
        self.get_logger().info('=' * 60)

        # Navigator
        self.nav = nav
        # Controller client
        self.client = self.create_client(ExplorationControl, '/snc_exploration_control')

        # Configure parameters with defaults
        params = params or {}
        
        self.pose_sample_interval_s = params.get('pose_sample_interval_s', 0.5)
        self.waypoint_spacing_min = params.get('waypoint_spacing_min', 0.15)
        self.waypoint_rotation_min = math.radians(params.get('waypoint_rotation_min', 15))
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.explore_breadcrumbs = []  # List to store the breadcrumbs for the explore path
        self.return_breadcrumbs = []   # List to store the breadcrumbs for the return path
        self.return_path = None # Variable to store the final return path once complete
        self.last_recorded_pose = None
        self.last_recorded_yaw = None
        self.return_triggered = False # Flag to indicate if return home has been triggered, stops pose sampling when true

        # Publisher for /snc_status - Single source of truth for status messages
        self.pub_status = self.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE
        )

        # Start challenge subscription
        self.sub_start_challenge = self.create_subscription(
            START_CHALLENGE_INTERFACE,
            START_CHALLENGE_TOPIC,
            self.start_challenge_callback,
            TRIGGER_QOS
        )
        # Go home trigger subscription to start return path tracing
        self.sub_go_home = self.create_subscription(
            GO_HOME_INTERFACE,
            GO_HOME_TOPIC,
            self.home_trigger_callback,
            TRIGGER_QOS
        )
        # Contingency triggers
        self.sub_home_trigger = self.create_subscription(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            self.home_trigger_callback,
            TRIGGER_QOS
        )
        self.sub_start_trigger = self.create_subscription(
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            self.start_challenge_callback,
            TRIGGER_QOS
        )
        self.sub_teleop_trigger = self.create_subscription(
            TRIGGER_TELEOP_INTERFACE,
            TRIGGER_TELEOP_TOPIC,
            self.teleop_trigger_callback,
            TRIGGER_QOS
        )
        # Publisher for /path_explore to publish the path taken during exploration 
        # for assessors to evaluate
        self.pub_path_explore = self.create_publisher(
            PATH_EXPLORE_INTERFACE,
            PATH_EXPLORE_TOPIC,
            PATH_EXPLORE_BUFFER_SIZE
        )
        # Publisher for /breadcrumbs_return to publish the path taken during return
        # for assessors to evaluate
        self.pub_path_return = self.create_publisher(
            PATH_RETURN_INTERFACE,
            PATH_RETURN_TOPIC,
            PATH_RETURN_BUFFER_SIZE
        )

        self.sample_pose_timer = self.create_timer(self.pose_sample_interval_s, self.sample_pose_callback)
    
    def teleop_trigger_callback(self, _):
        """Callback for the teleop trigger, which allows manual control of the robot without path tracing.
        """
        self.get_logger().info("Teleop trigger received.")
        self.stop_exploration()
    
    def start_challenge_callback(self, _):
        """Callback for the start challenge trigger, which allows starting the path tracing without waiting for the transform to become available. Useful for testing.
        """
        self.get_logger().info('-' * 60)
        self.get_logger().info('PHASE 2: PATH TRACING STARTED')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Start challenge trigger received. Beginning path exploration...')
        # Start the pose sampling timer immediately without waiting for TF
        self.sample_pose_timer.reset()
        self.start_exploration()
        self.get_logger().info('  ✓ Path exploration active - recording waypoints')

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
            self.get_logger().debug(f"Pose sampling skipped or failed (skipped={pose == SAMPLE_SKIPPED}, failed={pose == SAMPLE_FAILED})")
            return
            
        # Save the waypoint to the appropriate breadcrumb list and publish the path
        if self.return_triggered:
            self.return_breadcrumbs.append(pose)
            self.get_logger().info(f'  Return: {len(self.return_breadcrumbs)} waypoints recorded')
            if len(self.return_breadcrumbs) > 0:  # Only publish if we have waypoints
                # Use the header from the most recent breadcrumb and update the timestamp to current
                path_msg = Path(header=self.return_breadcrumbs[-1].header, poses=self.return_breadcrumbs)
                path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_return.publish(path_msg)
        else:
            self.explore_breadcrumbs.append(pose)
            self.get_logger().info(f'  Explore: {len(self.explore_breadcrumbs)} waypoints recorded')
            if len(self.explore_breadcrumbs) > 0:  # Only publish if we have waypoints
                # Use the header from the most recent breadcrumb and update the timestamp to current
                path_msg = Path(header=self.explore_breadcrumbs[-1].header, poses=self.explore_breadcrumbs)
                path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_path_explore.publish(path_msg)

    def get_robot_pose_in_map_frame(self, tf_buffer=None, clock=None):
        """
        Get the robot's pose in the map frame.
        
        Args:
            tf_buffer: Optional tf buffer to use (for testing)
            clock: Optional clock to use (for testing)
            
        Returns:
            PoseStamped object or SAMPLE_FAILED/SAMPLE_SKIPPED constants
        """
        try:
            # Get current position
            t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time()) \
                if tf_buffer else self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Failed to get robot pose despite TF being available: {e}")
            return SAMPLE_FAILED
        
        current_x = t.transform.translation.x
        current_y = t.transform.translation.y

        # Check if waypoint minimums are satisfied before storing
        if self.last_recorded_pose is not None and self.last_recorded_yaw is not None:
            # Use the core module function for waypoint filtering logic
            yaw = self.get_yaw_from_transform(t)
            if not should_record_waypoint(
                current_x=current_x,
                current_y=current_y,
                current_yaw=yaw,
                last_recorded_pose=self.last_recorded_pose,
                last_recorded_yaw=self.last_recorded_yaw,
                waypoint_spacing_min=self.waypoint_spacing_min,
                waypoint_rotation_min=self.waypoint_rotation_min
            ):
                return SAMPLE_SKIPPED

        # Convert to pose stamped using the core module function
        pose = construct_pose_stamped(t, clock or self.get_clock(), 'map')

        # Update last recorded pose and yaw
        self.last_recorded_pose = (current_x, current_y)
        self.last_recorded_yaw = self.get_yaw_from_transform(t)
        return pose

    def get_yaw_from_transform(self, t):
        """
        Extracts the yaw (Z-axis rotation) from a geometry_msgs/TransformStamped.
        """
        # Extract quaternion components from the transform
        quaternion = (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        )

        # Convert quaternion to roll, pitch, yaw
        # returns a list
        euler = euler_from_quaternion(quaternion)

        # Return the yaw
        return euler[2]
    
    def home_trigger_callback(self, _):
        """
        The single point of truth for the 'Home' event.
        
        This callback coordinates the shutdown sequence:
        1. Update status message
        2. Stop exploration (awaiting completion) 
        3. Start return trajectory following
        """
        self.get_logger().info('-' * 60)
        self.get_logger().info('PHASE 3: RETURN HOME TRIGGERED')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Home trigger received. Coordinating return sequence...')
        
        self.pub_status.publish(String(data="STOPPING FOR HOME"))

        self.stop_exploration()
        self.get_logger().info('Exploration stopped, calculating return trajectory...')
        self.start_return_sequence()

    def start_return_sequence(self):
        """
        Calculate the return trajectory and begin navigation home.
        """
        self.return_triggered = True
        self.get_logger().info("Starting return trajectory following.")
        
        # Reset last recorded pose and yaw to ensure the first return waypoint
        # is recorded regardless of distance/rotation from the last explore waypoint
        self.last_recorded_pose = None
        self.last_recorded_yaw = None

        # Calculate the return trajectory and handle log failures
        self.get_logger().info(f'  Calculating return trajectory from {len(self.explore_breadcrumbs)} explore waypoints...')
        return_trajectory = calculate_return_trajectory(self.explore_breadcrumbs, self.waypoint_spacing_min, self.waypoint_rotation_min)
        if return_trajectory is not None:
            self.return_path = Path(header=return_trajectory[0].header, poses=return_trajectory)
            self.get_logger().info(f'  ✓ Return trajectory calculated ({len(return_trajectory)} waypoints)')
        else:
            self.get_logger().error("Failed to calculate return trajectory, no path will be published")
            return

        # Small delay to let the controllers settle
        self.get_clock().sleep_for(Duration(seconds=.5))
        
        self.get_logger().info('  Navigating Home...')
        self.nav.followPath(self.return_path)

    
    def wait_for_robot_pose(self):
        """Wait for the robot pose transform to become available."""
        self.get_logger().info("Pre flight check: Waiting for (robot pose) map to base_link transform...")
        
        # Block until the transform is available or 5 seconds pass
        while not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=5.0)):
            self.get_logger().info("Still waiting for TF...")
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Transform found! Starting path tracing.")

    def _call_service(self, command):
        """
        Internal helper to call the exploration service.
        
        Args:
            command: The command string (START, STOP, RESUME, TELEOP)
            
        Returns:
            The service response or None if service call fails
        """

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service not available: {command}")
            return None
        
        request = ExplorationControl.Request()
        request.command = command
        
        # Use synchronous service call to ensure the service completes before returning
        future = self.client.call_async(request)
        
        # Wait for the service to complete
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(f"Service call '{command}' succeeded: {response.message}")
                    return response
                except Exception as e:
                    self.get_logger().error(f"Service call '{command}' failed: {e}")
                    return None
            # Spin briefly to process the service response
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return None

    def stop_exploration(self):
        """Stops the exploration process."""
        self.get_logger().info("Requesting exploration STOP...")
        return self._call_service("STOP")

    def start_exploration(self):
        """Starts the exploration process without blocking."""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available: START")
            return
        
        request = ExplorationControl.Request()
        request.command = "START"
        
        # This returns a future immediately and does NOT block
        self.future = self.client.call_async(request)
        
        # Add a callback to handle the result when it eventually arrives
        self.future.add_done_callback(self.exploration_response_callback)
        self.get_logger().info("Requesting exploration START (async)...")

    def exploration_response_callback(self, future):
        """Handle the result of the service call once it returns."""
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def resume(self):
        """Resumes the exploration process, allowing it to continue from where it left off."""
        self.get_logger().info("Requesting exploration RESUME...")
        return self._call_service("RESUME")
    
    def teleop(self):
        """Switches to teleop control."""
        self.get_logger().info("Requesting teleop control...")
        return self._call_service("TELEOP")

def main(args=None):
    rclpy.init(args=args)
    
    nav = BasicNavigator(node_name='path_tracing_node')
    node = PathTracingNode(nav)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

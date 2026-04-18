#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, TransformException
from nav2_simple_commander.robot_navigator import BasicNavigator
from snc.exploration_control import ExplorationController

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
    TRIGGER_HOME_TOPIC, TRIGGER_HOME_BUFFER_SIZE, TRIGGER_HOME_INTERFACE,
    SNC_STATUS_TOPIC, SNC_STATUS_INTERFACE, SNC_STATUS_BUFFER_SIZE
)
import math
import tf_transformations


# Import core functions for easier testing
from snc.path_tracing_core import (
    should_record_waypoint,
    construct_pose_stamped,
    get_yaw_from_transform,
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
        self.get_logger().info('Path tracing node launched')

        # Navigator
        self.nav = nav
        
        # Initialize the ExplorationController (logic-only, no ROS interfaces)
        self.exploration_controller = ExplorationController(self)

        from rclpy.callback_groups import ReentrantCallbackGroup
        self.cb_group = ReentrantCallbackGroup()

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

        # === ROS INTERFACES (All handled by this node) ===

        # Publisher for /snc_status - Single source of truth for status messages
        self.pub_status = self.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE
        )

        # Go home trigger subscription to start return path tracing
        self.sub_go_home = self.create_subscription(
            GO_HOME_INTERFACE,
            GO_HOME_TOPIC,
            self.home_trigger_callback,
            GO_HOME_BUFFER_SIZE,
            callback_group=self.cb_group
        )        
        # Contingency Return home trigger
        self.sub_home_trigger = self.create_subscription(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            self.home_trigger_callback,
            TRIGGER_HOME_BUFFER_SIZE,
            callback_group=self.cb_group
        )
        # Publisher for /path_explore to publish the path taken during exploration 
        # for assessors to evaluate
        self.pub_path_explore = self.create_publisher(
            PATH_EXPLORE_INTERFACE,
            PATH_EXPLORE_TOPIC,
            PATH_EXPLORE_BUFFER_SIZE
        )
        # # Publisher for /breadcrumbs_explore to publish the breadcrumbs taken during exploration
        # # for Node 1 to improve exploration
        # self.pub_explore_breadcrumbs = self.create_publisher(
        #     EXPLORE_BREADCRUMBS_INTERFACE,
        #     EXPLORE_BREADCRUMBS_TOPIC,
        #     EXPLORE_BREADCRUMBS_BUFFER_SIZE
        # )
        # Publisher for /breadcrumbs_return to publish the path taken during return
        # for assessors to evaluate
        self.pub_path_return = self.create_publisher(
            PATH_RETURN_INTERFACE,
            PATH_RETURN_TOPIC,
            PATH_RETURN_BUFFER_SIZE
        )

        self.sample_pose_timer = self.create_timer(self.pose_sample_interval_s, self.sample_pose_callback, callback_group=self.cb_group)
    
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
            self.get_logger().info(f"Stored return waypoint {len(self.return_breadcrumbs)}")
            self.pub_path_return.publish(Path(header=pose.header, poses=self.return_breadcrumbs))
        else:
            self.explore_breadcrumbs.append(pose)
            self.get_logger().info(f"Stored explore waypoint {len(self.explore_breadcrumbs)}")
            self.pub_path_explore.publish(Path(header=pose.header, poses=self.explore_breadcrumbs))

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
        euler = tf_transformations.euler_from_quaternion(quaternion)

        # Return the yaw
        return euler[2]
    
    async def home_trigger_callback(self, _):
        """
        The single point of truth for the 'Home' event.
        
        This callback coordinates the shutdown sequence:
        1. Update status message
        2. Stop exploration (awaiting completion)
        3. Start return trajectory following
        """
        self.get_logger().info("Home trigger received. Coordinating shutdown...")
        
        # 1. Update Status
        self.pub_status.publish(String(data="STOPPING FOR HOME"))

        # 2. Tell Controller to stop exploration
        # We await this to ensure exploration is DEAD before we start navigating
        await self.exploration_controller.stop()

        # 3. Trigger the internal return-to-home logic
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
        return_trajectory = calculate_return_trajectory(self.explore_breadcrumbs, self.waypoint_spacing_min, self.waypoint_rotation_min)
        if return_trajectory is not None:
            self.return_path = Path(header=return_trajectory[0].header, poses=return_trajectory)
        else:
            self.get_logger().error("Failed to calculate return trajectory, no path will be published")
            return

        # Small delay to let the controllers settle
        self.get_clock().sleep_for(Duration(seconds=.5))
        
        self.get_logger().info('Navigating Home...')
        self.nav.followPath(self.return_path)

    
    def wait_for_robot_pose(self):
        """Wait for the robot pose transform to become available."""
        self.get_logger().info("Pre flight check: Waiting for (robot pose) map to base_link transform...")
        
        # Block until the transform is available or 5 seconds pass
        while not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=5.0)):
            self.get_logger().info("Still waiting for TF...")
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Transform found! Starting path tracing.")

def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    
    nav = BasicNavigator(node_name='path_tracing_node')
    node = PathTracingNode(nav) 
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(nav)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

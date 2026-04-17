import rclpy
from snc.navigation_node import NavigationNode
from nav_msgs.msg import Path
from snc.constants import (
    TRIGGER_HOME_TOPIC, TRIGGER_HOME_BUFFER_SIZE, TRIGGER_HOME_INTERFACE, 
    TRIGGER_START_TOPIC, TRIGGER_START_BUFFER_SIZE, TRIGGER_START_INTERFACE,
    TRIGGER_TELEOP_TOPIC, TRIGGER_TELEOP_BUFFER_SIZE, TRIGGER_TELEOP_INTERFACE,
    SNC_STATUS_TOPIC, SNC_STATUS_INTERFACE, SNC_STATUS_BUFFER_SIZE
)
class ExplorationController:
    def __init__(self, nav):
        self.nav = nav
        self.client = self.nav.create_client(NavigationNode, '/snc_exploration_control')

        # Subscriptions for /trigger_start, /trigger_home, /trigger_teleop
        self.sub_trigger_start = self.create_subscription(
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            self.start_trigger_callback,
            TRIGGER_START_BUFFER_SIZE
        )
        self.sub_trigger_teleop = self.create_subscription(
            TRIGGER_TELEOP_INTERFACE,
            TRIGGER_TELEOP_TOPIC,
            self.teleop_trigger_callback,
            TRIGGER_TELEOP_BUFFER_SIZE
        )
        self.sub_trigger_home = self.create_subscription(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            self.home_trigger_callback,
            TRIGGER_HOME_BUFFER_SIZE
        )

        # Publisher for SNC status updates
        self.pub_snc_status = self.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE
        )

        # Wait for the Navigation Node to be available before proceeding
        self.wait_for_service()

    def wait_for_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.nav.get_logger().info('Exploration service not available, waiting...')

    def __control_exploration(self, command_string):
        """
        Sends a START or STOP command to the exploration service.
        """
        request = NavigationNode.Request()
        request.command = command_string  # "START" or "STOP"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.nav, future)
        
        return future.result()
    
    def start(self):
        """Starts the exploration process with all frontiers unexplored."""
        self.nav.get_logger().info("Starting exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="EXPLORING"))
        return self.__control_exploration("START")

    def stop(self):
        """Stops the exploration process."""
        self.nav.get_logger().info("Stopping exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="STOPPING EXPLORATION"))
        return self.__control_exploration("STOP")

    def resume(self):
        """Resumes the exploration process, allowing it to continue from where it left off."""
        self.nav.get_logger().info("Resuming exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="EXPLORING"))
        return self.__control_exploration("RESUME")
    
    def teleop(self):
        """Switches to teleop control."""
        self.nav.get_logger().info("Switching to teleop control...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="TELEOP OVERRIDE"))
        return self.__control_exploration("TELEOP")
    
    def home_trigger_callback(self, _):
        """Callback function for the home trigger subscription."""
        self.nav.get_logger().info("Home trigger received")
        self.stop()
    
    def teleop_trigger_callback(self, _):
        """Callback function for the teleop trigger subscription."""
        self.nav.get_logger().info("Teleop trigger received")
        self.teleop()
    
    def start_trigger_callback(self, _):
        """Callback function for the start trigger subscription."""
        self.nav.get_logger().info("Start trigger received")
        self.start()
    

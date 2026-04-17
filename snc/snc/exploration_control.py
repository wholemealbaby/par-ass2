import rclpy
from snc_interfaces.srv import ExplorationControl
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
        self.client = self.nav.create_client(ExplorationControl, '/snc_exploration_control')
        self.logger = self.nav.get_logger().get_child('ExplorationController')

        # Use a callback group to ensure callbacks are processed in the same thread
        self.cb_group = self.nav.default_callback_group()
        # Subscriptions for /trigger_start, /trigger_home, /trigger_teleop
        self.sub_trigger_start = self.nav.create_subscription(
            TRIGGER_START_INTERFACE,
            TRIGGER_START_TOPIC,
            self.start_trigger_callback,
            TRIGGER_START_BUFFER_SIZE,
            callback_group=self.cb_group
        )
        self.sub_trigger_teleop = self.nav.create_subscription(
            TRIGGER_TELEOP_INTERFACE,
            TRIGGER_TELEOP_TOPIC,
            self.teleop_trigger_callback,
            TRIGGER_TELEOP_BUFFER_SIZE,
            callback_group=self.cb_group
        )
        self.sub_trigger_home = self.nav.create_subscription(
            TRIGGER_HOME_INTERFACE,
            TRIGGER_HOME_TOPIC,
            self.home_trigger_callback,
            TRIGGER_HOME_BUFFER_SIZE,
            callback_group=self.cb_group
        )

        self.pub_snc_status = self.nav.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE
        )

        # Publisher for SNC status updates
        self.pub_snc_status = self.nav.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE
        )

        # Wait for the Navigation Node to be available before proceeding
        self.wait_for_service()

    def wait_for_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Exploration service not available, waiting...')

    async def __control_exploration(self, command_string):
        """
        Sends a START or STOP command to the exploration service.
        """
        request = ExplorationControl.Request()
        request.command = command_string

        self.logger.info(f"Sending command: {command_string}")
        future = self.client.call_async(request)
        

        response = await future
        return response
    
    async def start(self):
        """Starts the exploration process with all frontiers unexplored."""
        self.logger.info("Starting exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="EXPLORING"))
        return await self.__control_exploration("START")

    async def stop(self):
        """Stops the exploration process."""
        self.logger.info("Stopping exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="STOPPING EXPLORATION"))
        return await self.__control_exploration("STOP")

    async def resume(self):
        """Resumes the exploration process, allowing it to continue from where it left off."""
        self.logger.info("Resuming exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="EXPLORING"))
        return await self.__control_exploration("RESUME")
    
    async def teleop(self):
        """Switches to teleop control."""
        self.logger.info("Switching to teleop control...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="TELEOP OVERRIDE"))
        return await self.__control_exploration("TELEOP")
    
    async def home_trigger_callback(self, _):
        """Callback function for the home trigger subscription."""
        self.logger.info("Home trigger received")
        await self.stop()
    
    async def teleop_trigger_callback(self, _):
        """Callback function for the teleop trigger subscription."""
        self.logger.info("Teleop trigger received")
        await self.teleop()
    
    async def start_trigger_callback(self, _):
        """Callback function for the start trigger subscription."""
        self.logger.info("Start trigger received")
        await self.start()
    

import rclpy
from rclpy.node import Node
from snc_interfaces.srv import ExplorationControl
from nav_msgs.msg import Path
from snc.constants import (
    TRIGGER_HOME_TOPIC, TRIGGER_HOME_BUFFER_SIZE, TRIGGER_HOME_INTERFACE, 
    TRIGGER_START_TOPIC, TRIGGER_START_BUFFER_SIZE, TRIGGER_START_INTERFACE,
    TRIGGER_TELEOP_TOPIC, TRIGGER_TELEOP_BUFFER_SIZE, TRIGGER_TELEOP_INTERFACE,
    SNC_STATUS_TOPIC, SNC_STATUS_INTERFACE, SNC_STATUS_BUFFER_SIZE
)

class ExplorationController(Node):
    def __init__(self):
        # Initialize the Node with a name
        super().__init__('exploration_controller')
        
        # Create Service Client
        self.client = self.create_client(ExplorationControl, '/snc_exploration_control')

        # Subscriptions
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

        # Publisher
        self.pub_snc_status = self.create_publisher(
            SNC_STATUS_INTERFACE,
            SNC_STATUS_TOPIC,
            SNC_STATUS_BUFFER_SIZE
        )

        # Wait for the service to be ready
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Exploration service not available, waiting...')
        
        self.get_logger().info("Exploration Controller Node Initialized.")

    def __control_exploration(self, command_string):
        """
        Sends a command to the exploration service asynchronously.
        """
        request = ExplorationControl.Request()
        request.command = command_string

        # Using call_async to prevent deadlocking the executor
        future = self.client.call_async(request)
        future.add_done_callback(self._service_response_callback)

    def _service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call successful")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def start(self):
        self.get_logger().info("Starting exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="EXPLORING"))
        self.__control_exploration("START")

    def stop(self):
        self.get_logger().info("Stopping exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="STOPPING EXPLORATION"))
        self.__control_exploration("STOP")

    def resume(self):
        self.get_logger().info("Resuming exploration...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="EXPLORING"))
        self.__control_exploration("RESUME")
    
    def teleop(self):
        self.get_logger().info("Switching to teleop control...")
        self.pub_snc_status.publish(SNC_STATUS_INTERFACE(data="TELEOP OVERRIDE"))
        self.__control_exploration("TELEOP")
    
    # Callbacks
    def home_trigger_callback(self, msg):
        self.get_logger().info("Home trigger received")
        self.stop()
    
    def teleop_trigger_callback(self, msg):
        self.get_logger().info("Teleop trigger received")
        self.teleop()
    
    def start_trigger_callback(self, msg):
        self.get_logger().info("Start trigger received")
        self.start()

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
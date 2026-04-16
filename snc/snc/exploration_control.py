import rclpy
from snc.ExplorationNode import ExplorationControl

class ExplorationController:
    def __init__(self, nav):
        self.nav = nav
        self.client = self.nav.create_client(ExplorationControl, '/snc_exploration_control')
        self.wait_for_service()

    def wait_for_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.nav.get_logger().info('Exploration service not available, waiting...')

    def control_exploration(self, command_string):
        """
        Sends a START or STOP command to the exploration service.
        """
        request = ExplorationControl.Request()
        request.command = command_string  # "START" or "STOP"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.nav, future)
        
        return future.result()

# --- Example Usage in your Overriding Logic ---

# controller = ExplorationController(nav)

# 1. Stop Exploration
# nav.get_logger().info("Stopping exploration...")
# controller.control_exploration("STOP")

# 2. Brief sleep with the node clock to ensure velocity commands have ceased
# nav.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

# 3. Take over with Nav2
# nav.followPath(my_priority_path)
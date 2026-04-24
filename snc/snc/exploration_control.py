from snc_interfaces.srv import ExplorationControl

# DEPRECATED
# DO NOT USE

class ExplorationController:
    """
    A logic-only class that handles exploration control service calls.
    
    This class does NOT create any subscriptions or publishers. All ROS
    interfaces are handled by the PathTracingNode (the coordinator).
    
    The controller is passed the node reference to create service clients
    and use the node's logger for consistent logging.
    """
    def __init__(self, node):
        """
        Initialize the ExplorationController.
        
        Args:
            node: The PathTracingNode instance (the coordinator) used to create
                  service clients and for logging.
        """
        self.node = node
        self.client = self.node.create_client(ExplorationControl, '/snc_exploration_control')
        self.logger = self.node.get_logger().get_child('ExplorationController')
        
        # Wait for the service to be available
        self.wait_for_service()

    def wait_for_service(self):
        """Wait for the exploration control service to become available."""
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Exploration service not available, waiting...')

    def _call_service(self, command):
        """
        Internal helper to call the exploration service.
        
        Args:
            command: The command string (START, STOP, RESUME, TELEOP)
            
        Returns:
            The service response or None if service call fails
        """
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f"Service not available for command: {command}")
            return None
        
        request = ExplorationControl.Request()
        request.command = command
        
        self.logger.info(f"Sending command: {command}")
        response = self.client.call(request)
        return response

    def stop(self):
        """Stops the exploration process."""
        self.logger.info("Requesting exploration STOP...")
        return self._call_service("STOP")

    def start(self):
        """Starts the exploration process with all frontiers unexplored."""
        self.logger.info("Requesting exploration START...")
        return self._call_service("START")

    def resume(self):
        """Resumes the exploration process, allowing it to continue from where it left off."""
        self.logger.info("Requesting exploration RESUME...")
        return self._call_service("RESUME")
    
    def teleop(self):
        """Switches to teleop control."""
        self.logger.info("Requesting teleop control...")
        return self._call_service("TELEOP")

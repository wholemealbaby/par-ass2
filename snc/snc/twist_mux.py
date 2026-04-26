#!/usr/bin/env python3
"""
Twist Mux Node with Testing Mode Lock

This node acts as a central mux for multiple cmd_vel sources and provides
a testing mode lock that blocks all movement commands when enabled.

Usage:
    - When testing_mode=True: All Twist commands are blocked
    - When testing_mode=False: Commands are passed through normally

Input Topics:
    - /cmd_vel_nav (default) - Nav2 navigation source (remapped from /cmd_vel)
    - /cmd_vel_raw - Manual navigation commands (from NavigationNode)
    - /cmd_vel_teleop - Teleop keyboard/joystick input
    - /cmd_vel_manual - Manual override input

Output Topic:
    - /cmd_vel - Only published when testing_mode=False (robot motor input)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

"""DISABLED AND DEPRECATED AFTER DEMO: 
This node was used during development to safely test the Nav2 integration by 
blocking all movement commands when testing_mode=True."""

class TwistMuxNode(Node):
    """
    Twist mux node with testing mode lock.
    
    This node subscribes to multiple cmd_vel sources and only publishes
    authorized commands when testing_mode is False. When testing_mode=True,
    all Twist messages are blocked to prevent robot movement during testing.
    """
    
    def __init__(self):
        super().__init__('twist_mux_node')
        self.get_logger().info('Twist mux node launched')
        
        # Declare parameters with defaults
        self.declare_parameter(
            'testing_mode',
            False,
            ParameterDescriptor(
                name='testing_mode',
                type=ParameterType.PARAMETER_BOOL,
                description='When True, blocks all Twist commands to prevent movement',
                read_only=False
            )
        )
        
        self.declare_parameter(
            'lock_teleop',
            True,
            ParameterDescriptor(
                name='lock_teleop',
                type=ParameterType.PARAMETER_BOOL,
                description='When True, blocks teleop input even when testing_mode=False',
                read_only=False
            )
        )
        
        self.declare_parameter(
            'lock_manual',
            True,
            ParameterDescriptor(
                name='lock_manual',
                type=ParameterType.PARAMETER_BOOL,
                description='When True, blocks manual input even when testing_mode=False',
                read_only=False
            )
        )
        
        # Nav2 input topic (remapped from /cmd_vel)
        self.declare_parameter(
            'cmd_vel_nav_topic',
            '/cmd_vel_nav',
            ParameterDescriptor(
                name='cmd_vel_nav_topic',
                type=ParameterType.PARAMETER_STRING,
                description='Nav2 cmd_vel input topic (remapped from /cmd_vel)',
                read_only=False
            )
        )
        
        self.declare_parameter(
            'cmd_vel_raw_topic',
            '/cmd_vel_raw',
            ParameterDescriptor(
                name='cmd_vel_raw_topic',
                type=ParameterType.PARAMETER_STRING,
                description='Manual navigation cmd_vel input topic',
                read_only=False
            )
        )
        
        self.declare_parameter(
            'cmd_vel_teleop_topic',
            '/cmd_vel_teleop',
            ParameterDescriptor(
                name='cmd_vel_teleop_topic',
                type=ParameterType.PARAMETER_STRING,
                description='Teleop input topic',
                read_only=False
            )
        )
        
        self.declare_parameter(
            'cmd_vel_manual_topic',
            '/cmd_vel_manual',
            ParameterDescriptor(
                name='cmd_vel_manual_topic',
                type=ParameterType.PARAMETER_STRING,
                description='Manual override topic',
                read_only=False
            )
        )
        
        # Get parameter values
        self.testing_mode = self.get_parameter('testing_mode').value
        self.lock_teleop = self.get_parameter('lock_teleop').value
        self.lock_manual = self.get_parameter('lock_manual').value
        
        self.cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        self.cmd_vel_raw_topic = self.get_parameter('cmd_vel_raw_topic').value
        self.cmd_vel_teleop_topic = self.get_parameter('cmd_vel_teleop_topic').value
        self.cmd_vel_manual_topic = self.get_parameter('cmd_vel_manual_topic').value
        
        self.get_logger().info(f'testing_mode: {self.testing_mode}')
        self.get_logger().info(f'lock_teleop: {self.lock_teleop}')
        self.get_logger().info(f'lock_manual: {self.lock_manual}')
        self.get_logger().info(f'cmd_vel_nav_topic: {self.cmd_vel_nav_topic}')
        self.get_logger().info(f'cmd_vel_raw_topic: {self.cmd_vel_raw_topic}')
        
        # Track latest Twist messages from each source
        self.latest_cmd_vel_nav = None
        self.latest_cmd_vel_raw = None
        self.latest_teleop = None
        self.latest_manual = None
        
        # Track which sources have published
        self.has_cmd_vel_nav = False
        self.has_cmd_vel_raw = False
        self.has_teleop = False
        self.has_manual = False
        
        # Publisher for authorized cmd_vel (publishes to /cmd_vel to control the robot)
        self.pub_authorized = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Publisher for testing_mode status
        self.pub_testing_mode = self.create_publisher(
            Bool,
            '/twist_mux/testing_mode',
            10
        )
        
        # Subscribers for each input source
        self.sub_cmd_vel_nav = self.create_subscription(
            Twist,
            self.cmd_vel_nav_topic,
            self.cmd_vel_nav_callback,
            10
        )
        
        self.sub_cmd_vel_raw = self.create_subscription(
            Twist,
            self.cmd_vel_raw_topic,
            self.cmd_vel_raw_callback,
            10
        )
        
        self.sub_teleop = self.create_subscription(
            Twist,
            self.cmd_vel_teleop_topic,
            self.teleop_callback,
            10
        )
        
        self.sub_manual = self.create_subscription(
            Twist,
            self.cmd_vel_manual_topic,
            self.manual_callback,
            10
        )
        
        # Timer to check for stale messages and republish if needed
        # This ensures the latest command is always published when authorized
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Parameter callback to handle runtime parameter changes
        self.add_on_set_parameters_callback(self.on_parameter_change)
        
        self.get_logger().info('Twist mux initialized successfully')
    
    def on_parameter_change(self, params):
        """Handle parameter changes at runtime."""
        for param in params:
            if param.name == 'testing_mode':
                if param.type_ == Parameter.Type.BOOL:
                    self.testing_mode = param.value
                    self.get_logger().info(f'testing_mode changed to: {self.testing_mode}')
                    self.pub_testing_mode.publish(Bool(data=self.testing_mode))
            elif param.name == 'lock_teleop':
                if param.type_ == Parameter.Type.BOOL:
                    self.lock_teleop = param.value
                    self.get_logger().info(f'lock_teleop changed to: {self.lock_teleop}')
            elif param.name == 'lock_manual':
                if param.type_ == Parameter.Type.BOOL:
                    self.lock_manual = param.value
                    self.get_logger().info(f'lock_manual changed to: {self.lock_manual}')
        
        return rclpy.parameter.SetParametersResult(successful=True)
    
    def cmd_vel_nav_callback(self, msg: Twist) -> None:
        """Callback for Nav2 cmd_vel source."""
        self.latest_cmd_vel_nav = msg
        self.has_cmd_vel_nav = True
        self.get_logger().debug('Received cmd_vel_nav (Nav2)')
    
    def cmd_vel_raw_callback(self, msg: Twist) -> None:
        """Callback for manual navigation cmd_vel source."""
        self.latest_cmd_vel_raw = msg
        self.has_cmd_vel_raw = True
        self.get_logger().debug('Received cmd_vel_raw')
    
    def teleop_callback(self, msg: Twist) -> None:
        """Callback for teleop input."""
        self.latest_teleop = msg
        self.has_teleop = True
        self.get_logger().debug('Received teleop cmd_vel')
    
    def manual_callback(self, msg: Twist) -> None:
        """Callback for manual override input."""
        self.latest_manual = msg
        self.has_manual = True
        self.get_logger().debug('Received manual cmd_vel')
    
    def timer_callback(self) -> None:
        """Timer callback to check and publish authorized commands."""
        # Always publish testing_mode status
        self.pub_testing_mode.publish(Bool(data=self.testing_mode))
        
        # If testing_mode is True, block all commands
        if self.testing_mode:
            self.get_logger().debug('testing_mode=True, blocking all commands')
            return
        
        # Determine which command to publish based on priority
        # Priority: teleop > manual > cmd_vel_nav > cmd_vel_raw
        cmd_to_publish = None
        source_name = None
        
        if self.has_teleop and not self.lock_teleop:
            cmd_to_publish = self.latest_teleop
            source_name = 'teleop'
        elif self.has_manual and not self.lock_manual:
            cmd_to_publish = self.latest_manual
            source_name = 'manual'
        elif self.has_cmd_vel_nav:
            cmd_to_publish = self.latest_cmd_vel_nav
            source_name = 'nav2'
        elif self.has_cmd_vel_raw:
            cmd_to_publish = self.latest_cmd_vel_raw
            source_name = 'manual_nav'
        
        if cmd_to_publish is not None:
            self.pub_authorized.publish(cmd_to_publish)
            self.get_logger().debug(f'Published authorized cmd_vel from {source_name}')
    
    def destroy_node(self):
        """Clean up node resources."""
        self.get_logger().info('Twist mux node shutting down')
        super().destroy_node()


def main(args=None):
    """Main entry point for the twist mux node."""
    rclpy.init(args=args)
    
    node = TwistMuxNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

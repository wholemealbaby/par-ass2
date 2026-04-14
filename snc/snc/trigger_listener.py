#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from pynput import keyboard


class TriggerListener(Node):
    """ROS 2 node that listens for keyboard shortcuts and publishes trigger messages."""

    def __init__(self):
        super().__init__('trigger_listener')

        self.get_logger().info('Trigger listener node launched')

        # QoS profile with RELIABLE reliability for critical triggers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )

        # Publishers for trigger topics
        self.pub_start = self.create_publisher(
            Empty, '/trigger_start', qos_profile
        )
        self.pub_teleop = self.create_publisher(
            Empty, '/trigger_teleop', qos_profile
        )
        self.pub_home = self.create_publisher(
            Empty, '/trigger_home', qos_profile
        )
        self.pub_status = self.create_publisher(
            String, '/snc_status', qos_profile
        )

        # Keyboard listener in background thread
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()

        self.get_logger().info('Keyboard listener started. Press Shift+S, Shift+T, Shift+H, or Spacebar.')

    def on_key_press(self, key):
        """Handle key press events."""
        try:
            # Check for capital letters (Shift + key)
            if hasattr(key, 'char') and key.char is not None:
                if key.char.isupper():
                    self.handle_uppercase_key(key.char)
            elif key == keyboard.Key.space:
                self.handle_emergency_stop()
        except AttributeError:
            pass

    def on_key_release(self, key):
        """Handle key release events (not used but required by pynput)."""
        pass

    def handle_uppercase_key(self, char):
        """Handle uppercase letter key presses."""
        if char == 'S':
            self.trigger_start()
        elif char == 'T':
            self.trigger_teleop()
        elif char == 'H':
            self.trigger_home()

    def trigger_start(self):
        """Publish start trigger."""
        msg = Empty()
        self.pub_start.publish(msg)
        status_msg = String()
        status_msg.data = "STATUS: Start Trigger Received"
        self.pub_status.publish(status_msg)
        self.get_logger().info("STATUS: Start Trigger Received")

    def trigger_teleop(self):
        """Publish teleop trigger."""
        msg = Empty()
        self.pub_teleop.publish(msg)
        status_msg = String()
        status_msg.data = "STATUS: Teleop Mode Requested"
        self.pub_status.publish(status_msg)
        self.get_logger().info("STATUS: Teleop Mode Requested")

    def trigger_home(self):
        """Publish home trigger."""
        msg = Empty()
        self.pub_home.publish(msg)
        status_msg = String()
        status_msg.data = "STATUS: Return Home Triggered"
        self.pub_status.publish(status_msg)
        self.get_logger().info("STATUS: Return Home Triggered")

    def handle_emergency_stop(self):
        """Publish emergency stop message."""
        status_msg = String()
        status_msg.data = "EMERGENCY: OPERATOR STOP"
        self.pub_status.publish(status_msg)
        self.get_logger().warn("EMERGENCY: OPERATOR STOP")

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.get_logger().info('Shutting down trigger listener...')
        self.listener.stop()
        super().destroy_node()


def main():
    """Main entry point for the trigger listener node."""
    rclpy.init()

    node = TriggerListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

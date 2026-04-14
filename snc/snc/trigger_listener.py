#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
import sys
import select
import termios
import tty

class TriggerListener(Node):
    def __init__(self):
        super().__init__('trigger_listener')

        self.get_logger().info('Trigger listener node launched (TTY Mode)')
        self.get_logger().info(
            '\nContingency Keyboard Shortcuts:\n'
            'S : Trigger Start\n'
            'T : Trigger Teleop\n'
            'H : Trigger Return Home\n'
            'Space : Emergency Stop\n'
            'Press Ctrl+C to exit'
        )

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)

        self.pub_start = self.create_publisher(Empty, '/trigger_start', qos_profile)
        self.pub_teleop = self.create_publisher(Empty, '/trigger_teleop', qos_profile)
        self.pub_home = self.create_publisher(Empty, '/trigger_home', qos_profile)
        self.pub_status = self.create_publisher(String, '/snc_status', qos_profile)

        # Save terminal settings to restore later
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Create a timer to check for keyboard input periodically
        self.timer = self.create_timer(0.1, self.check_keyboard)

    def get_key(self):
        """Reads a single keypress from stdin."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def check_keyboard(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = self.get_key()
            if key == 'S':
                self.trigger_start()
            elif key == 'T':
                self.trigger_teleop()
            elif key == 'H':
                self.trigger_home()
            elif key == ' ':
                self.handle_emergency_stop()
            elif key == '\x03': # Ctrl+C
                rclpy.shutdown()

    def trigger_start(self):
        self.pub_start.publish(Empty())
        self.publish_status("STATUS: Start Trigger Received")

    def trigger_teleop(self):
        self.pub_teleop.publish(Empty())
        self.publish_status("STATUS: Teleop Mode Requested")

    def trigger_home(self):
        self.pub_home.publish(Empty())
        self.publish_status("STATUS: Return Home Triggered")

    def handle_emergency_stop(self):
        self.publish_status("EMERGENCY: OPERATOR STOP", warn=True)

    def publish_status(self, text, warn=False):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

def main():
    rclpy.init()
    node = TriggerListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Crucial: Reset terminal settings so your shell isn't broken
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
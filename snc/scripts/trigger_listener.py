#!/usr/bin/env python3

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

# Key mapping configuration
KEY_MAPPING = {
    's': ('/trigger_start', "STATUS: Start Trigger Received"),
    't': ('/trigger_teleop', "STATUS: Teleop Mode Requested"),
    'h': ('/trigger_home', "STATUS: Return Home Triggered"),
    ' ': ('/snc_status', "EMERGENCY: OPERATOR STOP"),
}

class ScriptPublisher(Node):
    def __init__(self):
        super().__init__('trigger_script_client')
        self.publishers_dict = {}
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        
        # Pre-create publishers for speed
        for topic, _ in KEY_MAPPING.values():
            if topic not in self.publishers_dict:
                self.publishers_dict[topic] = self.create_publisher(Empty, topic, 10)

    def send_trigger(self, char):
        char_lower = char.lower()
        if char_lower in KEY_MAPPING:
            topic, log_text = KEY_MAPPING[char_lower]
            
            # Publish the trigger
            if topic == '/snc_status':
                msg = String(data=log_text)
                self.status_pub.publish(msg)
                print(f"\033[91m {log_text} \033[0m")
            else:
                self.publishers_dict[topic].publish(Empty())
                # Also log to status topic
                status_msg = String(data=log_text)
                self.status_pub.publish(status_msg)
                print(f"\033[92m [Sent] {char_lower.upper()} -> {topic} \033[0m")

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # Wait up to 0.1s for a keypress
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = ScriptPublisher()
    
    settings = termios.tcgetattr(sys.stdin)
    
    print("-" * 30)
    print(" ROBOT REMOTE CONTROL ACTIVE")
    print("-" * 30)
    print("S : Start")
    print("T : Teleop")
    print("H : Return Home")
    print("SPACE : Emergency Stop")
    print("CTRL+C : Exit")
    print("-" * 30)

    try:
        while True:
            key = get_key(settings)
            if key:
                if key == '\x03':  # Ctrl+C
                    break
                node.send_trigger(key)
            
            # Process any internal ROS business (like discovery)
            rclpy.spin_once(node, timeout_sec=0)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
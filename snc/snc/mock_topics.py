import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from find_object_2d.msg import ObjectsStamped
import time

# Attempt to import constants
try:
    import snc.constants as c
except ImportError:
    try:
        import constants as c
    except ImportError:
        class c:
            START_CHALLENGE_TOPIC = '/snc_start'
            OBJECTS_TOPIC = "/objectsStamped"
            PATH_RETURN_TOPIC = '/path_return'
            PATH_EXPLORE_TOPIC = '/path_explore'
            SNC_STATUS_TOPIC = '/snc_status'
            COVERAGE_TOPIC = '/covered_cells_marker'

class MockSNCNode(Node):
    def __init__(self):
        super().__init__('mock_snc_publisher')
        
        self.pub_start = self.create_publisher(Empty, c.START_CHALLENGE_TOPIC, 10)
        self.pub_status = self.create_publisher(String, c.SNC_STATUS_TOPIC, 10)
        self.pub_path = self.create_publisher(Path, c.PATH_EXPLORE_TOPIC, 10)
        self.pub_objects = self.create_publisher(ObjectsStamped, c.OBJECTS_TOPIC, 10)

    def run_test(self):
        print("Sending test messages...")

        # Send Empty (Start Signal)
        self.pub_start.publish(Empty())
        print(f"Sent: Empty to {c.START_CHALLENGE_TOPIC}")

        # Send String (Status)
        msg_status = String()
        msg_status.data = "Testing Recorder: All systems nominal."
        self.pub_status.publish(msg_status)
        print(f"Sent: String to {c.SNC_STATUS_TOPIC}")

        # Send Path (Navigation)
        msg_path = Path()
        msg_path.header.frame_id = "map"
        msg_path.header.stamp = self.get_clock().now().to_msg()
        # Add one dummy pose to the path
        dummy_pose = PoseStamped()
        dummy_pose.header = msg_path.header
        dummy_pose.pose.position.x = 1.0
        msg_path.poses.append(dummy_pose)
        self.pub_path.publish(msg_path)
        print(f"Sent: Path to {c.PATH_EXPLORE_TOPIC}")

        # Send ObjectsStamped (Vision)
        msg_obj = ObjectsStamped()
        msg_obj.header.stamp = self.get_clock().now().to_msg()
        # Mocking finding object ID 11 (Explosive)
        msg_obj.objects.data = [11.0, 100.0, 100.0, 50.0, 50.0]
        self.pub_objects.publish(msg_obj)
        print(f"Sent: ObjectsStamped to {c.OBJECTS_TOPIC}")

        print("\nTest sequence complete.")

def main():
    rclpy.init()
    node = MockSNCNode()
    
    # Give ROS a moment to discover the recorder node
    print("Waiting for discovery...")
    time.sleep(2) 
    
    node.run_test()
    
    # Spin briefly to ensure messages are physically sent over the wire
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
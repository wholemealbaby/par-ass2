import rclpy
from rclpy.node import Node
from find_object_2d.msg import ObjectsStamped
import math
from constants import HAZARD_SPEC_IDS_MAP, HAZARD_IMAGE_MAP

class Hazard:
    def __init__(self, msg):
        """Create a hazard object from a ObjectStamped message.
        Note that find object 2d often disguises ObjectStamped messages
        as Float32MultiArray interfaces"""

        # ID of the object in the find_object_2d session
        self.object_id = int(data[i])
        self.name = HAZARD_IMAGE_MAP[self.object_id]
        # ID of the hazard in the assignment specification
        self.id = HAZARD_SPEC_IDS_MAP[self.name] 
        self.width = data[i+1]
        self.height = data[i+2]

       # Position (from homography translation)
        dx = obj_data[i+9]
        dy = obj_data[i+10]

        # Rotation (Yaw) to Quaternion
        # h11 and h21 give us the rotation components
        h11 = obj_data[i+3]
        h21 = obj_data[i+6]
        yaw = math.atan2(h21, h11)

        # Construct the PoseStamped object
        ps = PoseStamped()
        
        # Sync header with the camera frame and time of detection
        ps.header = incoming_header
        
        # Position
        ps.pose.position.x = float(dx)
        ps.pose.position.y = float(dy)
        ps.pose.position.z = 0.0
        
        # Orientation (Quaternion for 2D Yaw)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self.camera_pose_stamped = ps



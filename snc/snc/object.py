import math

import numpy as np
from std_msgs.msg import Header
import math
from geometry_msgs.msg import PoseStamped
from snc.constants import OBJECT_MAP

class DetectedObject:
    def __init__(self, data_slice: list, header: Header):
        """
        Abstract representation of an object detected via find_object_2d.
        
        :param data_slice: 12 floats (ID, Width, Height, and 3x3 Matrix)
        :param header: The header from the incoming ROS message
        """
        self.header = header
        self.object_id = int(data_slice[0])
        self.name = OBJECT_MAP.get(self.object_id, "Unknown")
        self.ref_width = data_slice[1]
        self.ref_height = data_slice[2]

        # Reconstruct Homography Matrix
        # [h11, h12, h13, h21, h22, h23, h31, h32, h33]
        self.h_matrix = np.array(data_slice[3:12]).reshape(3, 3)

        # Calculate Center Point (Pixels)
        # multiply the center of the reference image by the matrix
        ref_center = np.array([self.ref_width / 2.0, self.ref_height / 2.0, 1.0])
        cam_center = np.dot(self.h_matrix, ref_center)
        
        # Normalize by the homogeneous coordinate (w)
        self.pixel_x = cam_center[0] / cam_center[2]
        self.pixel_y = cam_center[1] / cam_center[2]

        # Extract Yaw (Rotation around optical Z-axis)
        self.yaw = math.atan2(self.h_matrix[1, 0], self.h_matrix[0, 0])

        # Initialize Pose (Stored in camera optical frame for now)
        self.pose_stamped = self._create_camera_pose()

    def _create_camera_pose(self) -> PoseStamped:
        ps = PoseStamped()
        ps.header = self.header
        
        # These remain pixels until you apply your camera intrinsic projection
        ps.pose.position.x = float(self.pixel_x)
        ps.pose.position.y = float(self.pixel_y)
        ps.pose.position.z = 0.0 
        
        # Yaw to Quaternion (Rotation around Z)
        ps.pose.orientation.z = math.sin(self.yaw / 2.0)
        ps.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        return ps

    def get_map_pose(self, tf_buffer, target_frame="map", depth_value=1.0) -> PoseStamped:
        """
        Converts pixel coordinates to meters and transforms to the target frame.
        """
        # TODO: Implement the Pinhole Camera Model projection here
        # Then use tf_buffer.transform(self.pose_stamped, target_frame)
        pass

class ObjectHandler:
    def __init__(self):
        self.objects = []
    
    def add_object(self, data_slice: list, header) -> None:
        """
        Creates an Object instance and adds it to the internal list.
        """
        if len(data_slice) == 12:
            new_obj = DetectedObject(data_slice, header)
            self.objects.append(new_obj)
        else:
            # This handles cases where the array might be malformed
            print(f"Warning: Received data_slice of length {len(data_slice)}, expected 12.")

    def _validate_header(self, header) -> None:
        """Ensures the header contains necessary information
        including timestamp and frame_id.
        
        :param header: The header from the incoming ROS message"""

        if not isinstance(header, Header):
            raise TypeError("Expected header to be of type Header")
        if not header.stamp:
            raise ValueError("Header is missing timestamp")
        if not header.frame_id:
            raise ValueError("Header is missing frame_id")

    def add_objects_from_message(self, msg) -> None:
        """
        Parses the incoming message (likely ObjectsStamped) and populates the list.
        
        Note: msg.objects.data is the Float32MultiArray part.
        """
        
        # Access the underlying data array
        # Adjust 'msg.objects.data' if your message structure is different
        data = msg.objects.data 
        header = msg.header

        # Iterate through the flat array in steps of 12
        for i in range(0, len(data), 12):
            data_slice = data[i : i + 12]
            self.add_object(data_slice, header)
        
    def start_marker_detected(self) -> bool:
        """Checks if the 'Start' object is among the detected objects."""
        for obj in self.objects:
            if obj.name == "Start":
                return True
        return False
import math

import numpy as np
from snc.constants import HAZARD_MAP, OBJECT_MAP

import math
from geometry_msgs.msg import PoseStamped

class Hazard:
    def __init__(self, data_slice: list, header):
        """
        Create a hazard object from a slice of the ObjectStamped data.
        
        :param data_slice: A list of 12 floats representing one object
        :param header: The header from the original ObjectsStamped/Float32MultiArray message
        """
        # Basic Identity (Indices 0, 1, 2)
        self.object_id = int(data_slice[0])
        self.name = OBJECT_MAP.get(self.object_id, "unknown")
        self.id = HAZARD_MAP.get(self.name, -1) 
        
        self.width = data_slice[1]
        self.height = data_slice[2]

        # Reconstruct the 3x3 Homography Matrix (Indices 3-11)
        h = np.array(data_slice[3:12]).reshape(3, 3)

        # Calculate Center Point in Camera Frame
        # Define the center point of the reference image
        ref_center = np.array([self.width / 2.0, self.height / 2.0, 1.0])
        
        # Map the reference center to the camera frame: camera_pt = H * ref_pt
        cam_center = np.dot(h, ref_center)
        
        # Normalize by the third scale component (w)
        actual_x = cam_center[0] / cam_center[2]
        actual_y = cam_center[1] / cam_center[2]

        # Rotation (Yaw)
        yaw = math.atan2(h[1, 0], h[0, 0])

        # Construct PoseStamped
        ps = PoseStamped()
        ps.header = header
        
        # NOTE: These are still in PIXEL units. 
        # TODO: convert these to meters.
        ps.pose.position.x = float(actual_x)
        ps.pose.position.y = float(actual_y)
        ps.pose.position.z = 0.0
        
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.camera_pose_stamped = ps

class HazardManager:

    def __init__(self, msg):
        
        # The list of individual Hazard objects
        self.list = []
        
        # Parse the raw data (multiples of 12) from Float32MultiArray
        raw_data = msg.data
        for i in range(0, len(raw_data), 12):
            obj_slice = raw_data[i : i + 12]
            # Create Hazard instance and add to list
            self.list.append(Hazard(obj_slice, self.header))
            
        # Quick-access metadata
        self.count = len(self.list)
        self.any_detected = self.count > 0
    
    def get_hazard_by_name(self, name):
        """Helper to find a specific hazard by its name"""
        for h in self.list:
            if h.name == name:
                return h
        return None

    def get_unique_hazards(self):
        """Return a list of unique hazard names detected."""
        unique_names = set()
        unique_hazards = []
        for h in self.list:
            if h.name not in unique_names:
                unique_names.add(h.name)
                unique_hazards.append(h)
        return unique_hazards
    
    def get_hazard_by_object_id(self, object_id):
        """Helper to find a specific hazard by its object ID"""
        for h in self.list:
            if h.object_id == object_id:
                return h
        return None
    
    def get_hazard_by_id(self, spec_id):
        """Helper to find a specific hazard by its assignment ID"""
        for h in self.list:
            if h.id == spec_id:
                return h
        return None

    def get_hazard_by_id(self, spec_id):
        """Helper to find a specific hazard by its assignment ID"""
        for h in self.list:
            if h.id == spec_id:
                return h
        return None
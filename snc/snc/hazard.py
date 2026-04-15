import math
from constants import HAZARD_SPEC_IDS_MAP, HAZARD_IMAGE_MAP

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
        self.name = HAZARD_IMAGE_MAP.get(self.object_id, "unknown")
        self.id = HAZARD_SPEC_IDS_MAP.get(self.name, -1) 
        
        self.width = data_slice[1]
        self.height = data_slice[2]

        # Position (Indices 9 and 10)
        dx = data_slice[9]
        dy = data_slice[10]

        # Rotation (Indices 3 and 6)
        h11 = data_slice[3]
        h21 = data_slice[6]
        yaw = math.atan2(h21, h11)

        # Construct PoseStamped
        ps = PoseStamped()
        ps.header = header  # Carry over timestamp and frame_id
        
        ps.pose.position.x = float(dx)
        ps.pose.position.y = float(dy)
        ps.pose.position.z = 0.0
        
        # Yaw to Quaternion conversion
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.camera_pose_stamped = ps

class Hazards:
    def __init__(self, msg):
        """
        Container for all hazards detected in a single ROS 2 message.
        
        :param msg: The incoming ObjectsStamped message
        """
        # Metadata from the message header
        self.header = msg.header
        self.timestamp = msg.header.stamp
        self.frame_id = msg.header.frame_id
        
        # The list of individual Hazard objects
        self.list = []
        
        # Parse the raw data (multiples of 12)
        raw_data = msg.objects.data
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
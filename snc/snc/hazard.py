import math

import numpy as np
from snc.constants import HAZARD_MAP, OBJECT_MAP
from snc.object import DetectedObject
from std_msgs.msg import Header
import math
from geometry_msgs.msg import PoseStamped

class Hazard(DetectedObject):
    def __init__(self, data_slice: list, header: Header):
        """
        Represents a detected hazard object, extending the DetectedObject class.
        
        :param data_slice: 12 floats (ID, Width, Height, and 3x3 Matrix)
        :param header: The header from the incoming ROS message
        """
        super().__init__(data_slice, header)
        self.id = HAZARD_MAP.get(self.name, 0)  # Get hazard ID from name, default to 0 (Unknown)
    
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
    
    def get_hazard_by_id(self, spec_id) -> Hazard:
        """Helper to find a specific hazard by its assignment ID"""
        for h in self.list:
            if h.id == spec_id:
                return h
        raise ValueError(f"Hazard with ID {spec_id} not found")

    def get_hazard_by_id(self, spec_id) -> Hazard:
        """Helper to find a specific hazard by its assignment ID"""
        for h in self.list:
            if h.id == spec_id:
                return h
        raise ValueError(f"Hazard with ID {spec_id} not found")
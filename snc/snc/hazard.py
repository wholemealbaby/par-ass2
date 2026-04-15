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
    def __init__(self):
        # Using a set for rapid unique lookup (hazard IDs or names)
        self.hazards = set()
        
        # We can still keep a list for ordered access if needed
        self.list = []
        
        # Metadata
        self.count = 0
        self.any_detected = False

    def update_from_objects(self, detected_objects: list):
        """
        Receives a list of Object instances (from your ObjectManager/Handler)
        and converts them into Hazard objects.
        """
        # Clear current snapshot for fresh detection cycle
        self.list = []
        new_names = set()

        for obj in detected_objects:
            # Create Hazard from the abstract Object data
            # Assuming Hazard inherits from or wraps Object
            hazard = Hazard(obj, obj.header) 
            
            self.list.append(hazard, )
            new_names.add(hazard.name)

        # Update set and metadata
        self.hazards = new_names
        self.count = len(self.list)
        self.any_detected = self.count > 0

    def get_unique_hazards(self):
        """Returns the set of unique names currently detected."""
        return self.hazards

    def get_hazard_by_name(self, name):
        """Helper to find a specific hazard by its name."""
        for h in self.list:
            if h.name == name:
                return h
        return None

    def get_hazard_by_id(self, spec_id) -> 'Hazard':
        """Helper to find a specific hazard by its assignment ID."""
        for h in self.list:
            if h.id == spec_id:
                return h
        raise ValueError(f"Hazard with ID {spec_id} not found")

    def __len__(self):
        return self.count
import math

import numpy as np
from snc.constants import HAZARD_MAP, OBJECT_MAP
from snc.object import DetectedObject
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped


class Hazard(DetectedObject):
    def __init__(self, data_slice: list, header: Header):
        """
        Represents a detected hazard, extending DetectedObject with
        an assignment-spec hazard ID.

        :param data_slice: 12 floats (ID, Width, Height, and 3x3 homography matrix)
        :param header: The header from the incoming ROS message
        """
        super().__init__(data_slice, header)
        # Map the object name to the assignment hazard ID (0 = Unknown)
        self.id = HAZARD_MAP.get(self.name, 0)


class HazardManager:
    def __init__(self):
        # Set of unique hazard names seen so far (for fast membership tests)
        self.hazards = set()

        # Ordered list of Hazard objects from the most recent detection frame
        self.list = []

        self.count = 0
        self.any_detected = False

    def update_from_objects(self, detected_objects: list) -> None:
        """
        Convert a list of DetectedObject instances into Hazard objects and
        update the internal state for the current detection frame.

        :param detected_objects: list of DetectedObject from ObjectHandler
        """
        self.list = []
        new_names = set()

        for obj in detected_objects:
            hazard = Hazard(obj.raw_data, obj.header)
            self.list.append(hazard)
            new_names.add(hazard.name)

        self.hazards = new_names
        self.count = len(self.list)
        self.any_detected = self.count > 0

    def get_unique_hazards(self) -> set:
        """Return the set of unique hazard names currently detected."""
        return self.hazards

    def get_hazard_by_name(self, name: str) -> 'Hazard':
        """Find the first hazard matching name, or None."""
        for h in self.list:
            if h.name == name:
                return h
        return None

    def get_hazard_by_id(self, spec_id: int) -> 'Hazard':
        """Find the first hazard matching the assignment spec ID.

        :raises ValueError: if no matching hazard is found
        """
        for h in self.list:
            if h.id == spec_id:
                return h
        raise ValueError(f"Hazard with ID {spec_id} not found")

    def __len__(self):
        return self.count
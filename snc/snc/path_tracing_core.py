#!/usr/bin/env python3
"""
Core algorithmic logic for path tracing that can be tested independently of ROS2.
"""

import math
from geometry_msgs.msg import PoseStamped, TransformStamped


SAMPLE_FAILED = 0
SAMPLE_SKIPPED = 2


def calculate_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def calculate_yaw_delta(yaw1, yaw2):
    """Calculate absolute difference between two yaw angles."""
    return abs(yaw1 - yaw2)


def should_record_waypoint(
    current_x, 
    current_y, 
    current_yaw, 
    last_recorded_pose, 
    last_recorded_yaw,
    waypoint_spacing_min,
    waypoint_rotation_min
):
    """
    Determine whether to record a waypoint based on distance and rotation thresholds.
    
    Args:
        current_x: Current X coordinate
        current_y: Current Y coordinate  
        current_yaw: Current yaw angle
        last_recorded_pose: Previous (x, y) coordinates or None
        last_recorded_yaw: Previous yaw angle or None
        waypoint_spacing_min: Minimum distance threshold
        waypoint_rotation_min: Minimum rotation threshold in radians
        
    Returns:
        True if waypoint should be recorded, False if skipped
    """
    # If no previous pose, always record first waypoint
    if last_recorded_pose is None or last_recorded_yaw is None:
        return True
        
    # Calculate distance and yaw difference
    delta_dist = calculate_distance(
        current_x, current_y, 
        last_recorded_pose[0], last_recorded_pose[1]
    )
    delta_yaw = calculate_yaw_delta(current_yaw, last_recorded_yaw)
    
    # Skip if both thresholds are not met
    if delta_dist < waypoint_spacing_min and delta_yaw < waypoint_rotation_min:
        return False
        
    return True


def construct_pose_stamped(transform, clock, frame_id='map'):
    """
    Construct a PoseStamped message from a transform.
    
    Args:
        transform: TransformStamped object
        clock: Clock object to get current time
        frame_id: Frame ID for the pose header
        
    Returns:
        PoseStamped object
    """
    # Convert to pose stamped
    pose = PoseStamped()
    pose.header.stamp = clock.now().to_msg()
    pose.header.frame_id = frame_id
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = 0.0
    pose.pose.orientation = transform.transform.rotation
    
    return pose


def get_yaw_from_transform(t):
    """
    Extracts the yaw (Z-axis rotation) from a geometry_msgs/TransformStamped.
    
    Args:
        t: TransformStamped object
        
    Returns:
        Yaw angle in radians
    """
    # Extract quaternion components from the transform
    quaternion = (
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
    )

    # Convert quaternion to roll, pitch, yaw
    # returns a list
    euler = tf_transformations.euler_from_quaternion(quaternion)

    # Return the yaw
    return euler[2]


# Import here to avoid circular imports
try:
    import tf_transformations
except ImportError:
    # This module should only be imported when running tests
    pass
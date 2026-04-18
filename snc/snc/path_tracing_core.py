#!/usr/bin/env python3
"""
Core algorithmic logic for path tracing that can be tested independently of ROS2.
"""

import math
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf_transformations import euler_from_quaternion

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

def euler_from_quaternion_msg(q_msg):
    """
    Converts a ROS Quaternion message to Euler angles.
    """
    # Extract values from the ROS message
    q = [q_msg.x, q_msg.y, q_msg.z, q_msg.w]
    
    # Return (roll, pitch, yaw) in radians
    return euler_from_quaternion(q)

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
    try:
        euler = euler_from_quaternion(quaternion)
    except (ValueError, TypeError):
        # Handle potential NaN or invalid quaternion by returning 0
        return 0.0

    # Return the yaw
    return euler[2]


def reverse_waypoint_list(waypoints):
    """
    Reverse the stored list of PoseStamped messages.
    
    Args:
        waypoints: List of PoseStamped objects
        
    Returns:
        Reversed list of PoseStamped objects
    """
    return waypoints[::-1]


def invert_quaternions(waypoints):
    """
    Invert quaternions for each pose in the waypoint list so the robot faces 
    the direction of travel during return, not backwards.
    
    Args:
        waypoints: List of PoseStamped objects
        
    Returns:
        List of PoseStamped objects with inverted quaternions
    """
    inverted_waypoints = []
    
    for waypoint in waypoints:
        # Clone the waypoint
        new_waypoint = PoseStamped()
        new_waypoint.header = waypoint.header
        new_waypoint.pose.position = waypoint.pose.position
        
        # Invert the quaternion by negating z and w components
        # Or alternatively, we can compute the heading from consecutive waypoints
        q = (
            waypoint.pose.orientation.x,
            waypoint.pose.orientation.y,
            waypoint.pose.orientation.z,
            waypoint.pose.orientation.w
        )
        
        # Get Euler angles from quaternion
        roll, pitch, yaw = euler_from_quaternion(q)
        
        # To face forward along the return path, we need to rotate yaw by 180 degrees
        new_yaw = yaw + math.pi
        
        # Normalize yaw to [-pi, pi]
        while new_yaw > math.pi:
            new_yaw -= 2 * math.pi
        while new_yaw < -math.pi:
            new_yaw += 2 * math.pi
            
        # Convert back to quaternion
        new_q = quaternion_from_euler(roll, pitch, new_yaw)
        new_waypoint.pose.orientation = Quaternion(
            x=new_q[0],
            y=new_q[1], 
            z=new_q[2],
            w=new_q[3]
        )
        
        inverted_waypoints.append(new_waypoint)
    
    return inverted_waypoints

def normalize_angle(angle):
    # Reduce the angle to [-2pi, 2pi]
    angle = angle % (2 * math.pi)
    
    # Force into [-pi, pi]
    if angle > math.pi:
        angle -= 2 * math.pi
        
    return angle

def thin_waypoint_list(waypoints, waypoint_spacing_m, angle_threshold_deg):
    """
    Thin the waypoint list: if consecutive points are < waypoint_spacing_m apart, 
    skip intermediates to prevent Nav2 goal thrashing.
    
    Args:
        waypoints: List of PoseStamped objects
        waypoint_spacing_m: Minimum distance threshold between waypoints
        
    Returns:
        Thinned list of PoseStamped objects
    """
    if len(waypoints) <= 1:
        return waypoints
    
    thinned_waypoints = [waypoints[0]]  # Always keep the first waypoint
    
    thinned = [waypoints[0]]
    for i in range(1, len(waypoints)):
        last = thinned[-1]
        curr = waypoints[i]
        
        dist = calculate_distance(last.pose.position.x, last.pose.position.y, 
                                  curr.pose.position.x, curr.pose.position.y)
        
        # Calculate yaw diff
        _, _, last_yaw = euler_from_quaternion_msg(last.pose.orientation)
        _, _, curr_yaw = euler_from_quaternion_msg(curr.pose.orientation)
        angle_diff = abs(math.degrees(normalize_angle(curr_yaw - last_yaw)))

        # Keep if moved far enough OR turned significantly
        if dist >= waypoint_spacing_m or angle_diff >= angle_threshold_deg:
            thinned.append(curr)
       
    return thinned_waypoints

def calculate_return_trajectory(breadcrumbs, waypoint_spacing_m, angle_threshold_deg):
    """
    Calculate the return trajectory by reversing the explore breadcrumbs and 
    inverting the quaternions so the robot faces forward along the return path.
    
    Args:
        breadcrumbs: List of PoseStamped objects representing the explore path
    """
    reversed_breadcrumbs = reverse_waypoint_list(breadcrumbs)
    return thin_waypoint_list(invert_quaternions(reversed_breadcrumbs), waypoint_spacing_m, angle_threshold_deg)

# Import here to avoid circular imports
try:
    import tf_transformations
except ImportError:
    # This module should only be imported when running tests
    pass
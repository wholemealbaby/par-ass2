from std_msgs.msg import Empty, String
from nav_msgs.msg import Path
from find_object_2d.msg import ObjectsStamped

# Mapping of hazard names to their ID 
# from the assignment specification
HAZARD_MAP = {
    "Unknown": 0,
    "Explosive": 1,
    "Flammable Gas": 2,
    "Non-Flammable Gas": 3,
    "Dangerous When Wet": 4,
    "Flammable Solid": 5,
    "Spontaneously Combustible": 6,
    "Oxidizer": 7,
    "Organic Peroxide": 8,
    "Inhalation Hazard": 9,
    "Poison": 10,
    "Radioactive": 11,
    "Corrosive": 12
}

# This is the mapping from the object ID in the
# Find Object 2D configuration to the hazard name.
OBJECT_MAP = {
    "Start": 1,
    "Flammable Gas": 2,
    "Poison": 3,
    "Non-flammable Gas": 4,
    "Flammable Solid": 5,
    "Oxidizer": 6,
    "Inhalation Hazard": 7,
    "Corrosive": 8,
    "Organic Peroxide": 9,
    "Dangerous When Wet": 10,
    "Explosive": 11,
    "Radioactive": 12,
    "Spontaneously Combustible": 13,
}
# Create the reversals and update the original dictionaries
HAZARD_MAP.update({v: k for k, v in HAZARD_MAP.items()})
OBJECT_MAP.update({v: k for k, v in OBJECT_MAP.items()})

START_CHALLENGE_TOPIC = '/snc_start'
START_CHALLENGE_INTERFACE = Empty
START_CHALLENGE_BUFFER_SIZE = 1

OBJECTS_TOPIC = "/objectsStamped"
OBJECTS_BUFFER_SIZE = 20
OBJECTS_INTERFACE = ObjectsStamped

RETURN_BREADCRUMBS_TOPIC = '/breadcrumbs_return'
RETURN_BREADCRUMBS_BUFFER_SIZE = 10
RETURN_BREADCRUMBS_INTERFACE = Path

EXPLORE_BREADCRUMBS_TOPIC = '/breadcrumbs_explore'
EXPLORE_BREADCRUMBS_BUFFER_SIZE = 10
EXPLORE_BREADCRUMBS_INTERFACE = Path

RETURN_HOME_TRAJECTORY_TOPIC = '/return_home_trajectory'
RETURN_HOME_TRAJECTORY_BUFFER_SIZE = 1
RETURN_HOME_TRAJECTORY_INTERFACE = Path

HAZARD_SIGNAL_TOPIC = '/snc/hazard_signal'

ROBOT_POSE_TOPIC = '/snc/robot_pose'

HOME_TRIGGER_TOPIC = '/trigger_home'
HOME_TRIGGER_BUFFER_SIZE = 1
HOME_TRIGGER_INTERFACE = Path

# Trigger Listener Topics
TRIGGER_START_TOPIC = '/trigger_start'
TRIGGER_START_INTERFACE = Empty
TRIGGER_START_BUFFER_SIZE = 1

TRIGGER_TELEOP_TOPIC = '/trigger_teleop'
TRIGGER_TELEOP_INTERFACE = Empty
TRIGGER_TELEOP_BUFFER_SIZE = 1

TRIGGER_HOME_TOPIC = '/trigger_home'
TRIGGER_HOME_INTERFACE = Empty
TRIGGER_HOME_BUFFER_SIZE = 1

SNC_STATUS_TOPIC = '/snc_status'
SNC_STATUS_INTERFACE = String
SNC_STATUS_BUFFER_SIZE = 1

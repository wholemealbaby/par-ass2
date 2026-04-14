from std_msgs.msg import Empty, Float32MultiArray
from nav_msgs.msg import Path

hazard_names_map = {
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

START_CHALLENGE_TOPIC = '/snc_start'
START_CHALLENGE_INTERFACE = Empty
START_CHALLENGE_BUFFER_SIZE = 1

OBJECTS_TOPIC = ""
OBJECTS_BUFFER_SIZE = 20
OBJECTS_INTERFACE = Float32MultiArray

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

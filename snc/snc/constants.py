from std_msgs.msg import Empty, String
from nav_msgs.msg import Path
from find_object_2d.msg import ObjectsStamped
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

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
    "Non-Flammable Gas": 4,
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

TEST_SYNC_CHECK_TIMEOUT = 120.0  # seconds

# # Topic for Node 3 to pub all breadcrumbs taken during exploration
# # for Node 1 to improve exploration
# RETURN_BREADCRUMBS_TOPIC = '/breadcrumbs_return'
# RETURN_BREADCRUMBS_BUFFER_SIZE = 10
# RETURN_BREADCRUMBS_INTERFACE = Path

# Topic for Node 3 to pub the final return trajectory
# for assesors to evaluate
PATH_RETURN_TOPIC = '/path_return'
PATH_RETURN_BUFFER_SIZE = 10
PATH_RETURN_INTERFACE = Path

# # Topic for Node 3 to pub all breadcrumbs taken during exploration
# # for Node 1 to improve exploration
# EXPLORE_BREADCRUMBS_TOPIC = '/breadcrumbs_explore'
# EXPLORE_BREADCRUMBS_BUFFER_SIZE = 10
# EXPLORE_BREADCRUMBS_INTERFACE = Path

# Topic for Node 3 to pub the path taken during exploration
# for assesors to evaluate
PATH_EXPLORE_TOPIC = '/path_explore'
PATH_EXPLORE_BUFFER_SIZE = 10
PATH_EXPLORE_INTERFACE = Path

# # Topic for Node 3 to pub the final return trajectory
# RETURN_HOME_TRAJECTORY_TOPIC = '/return_home_trajectory'
# RETURN_HOME_TRAJECTORY_BUFFER_SIZE = 1
# RETURN_HOME_TRAJECTORY_INTERFACE = Path

HAZARD_SIGNAL_TOPIC = '/snc/hazard_signal'

ROBOT_POSE_TOPIC = '/snc/robot_pose'

# Go home signal topic to trigger return path tracing
GO_HOME_TOPIC = '/go_home'
GO_HOME_INTERFACE = Empty
GO_HOME_BUFFER_SIZE = 1

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

# Startup synchronization topic - all nodes publish their readiness here
STARTUP_SYNC_TOPIC = '/snc_startup_sync'
STARTUP_SYNC_INTERFACE = String
STARTUP_SYNC_BUFFER_SIZE = 1

# QoS profile for startup sync with latching to ensure late subscribers receive messages
STARTUP_SYNC_QOS = QoSProfile(
    depth=STARTUP_SYNC_BUFFER_SIZE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# QoS profile for trigger topics to match ros2 topic pub default behavior
TRIGGER_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Hazard marker topic for visualizing detected hazards
HAZARD_MARKER_TOPIC = '/hazards'
HAZARD_MARKER_BUFFER_SIZE = 10
HAZARD_MARKER_INTERFACE = Marker

COVERAGE_TOPIC = '/covered_cells_marker'
COVERAGE_BUFFER_SIZE = 1
COVERAGE_INTERFACE = Marker

# QoS profile for coverage marker to ensure RViz can subscribe correctly
COVERAGE_QOS = QoSProfile(
    depth=COVERAGE_BUFFER_SIZE,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
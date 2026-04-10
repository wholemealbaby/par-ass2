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

HAZARD_SIGNAL_TOPIC = '/snc/hazard_signal'
ROBOT_POSE_TOPIC = '/snc/robot_pose'
RETURN_WAYPOINTS_TOPIC = '/snc/return_waypoints'
HOME_TRIGGER_BUFFER_SIZE = 1
PATH_EXPLORE_BUFFER_SIZE = 10
PATH_RETURN_BUFFER_SIZE = 10
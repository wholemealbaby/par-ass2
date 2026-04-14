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
HOME_TRIGGER_TOPIC = '/trigger_home'
EXPLORE_BREADCRUMBS_TOPIC = '/breadcrumbs_explore'
RETURN_BREADCRUMBS_TOPIC = '/breadcrumbs_return'
RETURN_HOME_TRAJECTORY_TOPIC = '/return_home_trajectory'
HOME_TRIGGER_BUFFER_SIZE = 1
EXPLORE_BREADCRUMBS_BUFFER_SIZE = 10
RETURN_BREADCRUMBS_BUFFER_SIZE = 10
RETURN_HOME_TRAJECTORY_BUFFER_SIZE = 1
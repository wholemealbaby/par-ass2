# SNC Package - ROSBot Pro 3 Deployment Guide

## Overview

This documentation provides a comprehensive guide for deploying the SNC (Search and Clear Navigation) package on the ROSBot Pro 3 platform. The package implements a complete autonomous navigation system with hazard detection and path tracing capabilities.

## Project Architecture

The SNC system consists of four core ROS nodes that work together to provide autonomous exploration, hazard detection, and return navigation:

### Node 1: Navigation Control (`navigation_node`)
- **Purpose**: Main navigation controller for autonomous exploration
- **Responsibilities**:
  - Coordinates robot movement through waypoints using Nav2
  - Implements frontier-based exploration strategy
  - Manages exploration states (idle, exploring, spinning, returning, done)
  - Integrates with SLAM for mapping and localization

### Node 2: Marker Detection (`marker_detection_node`) 
- **Purpose**: Hazard marker detection and classification
- **Responsibilities**:
  - Processes camera feed to identify hazard markers
  - Classifies markers using predefined hazard categories
  - Transforms detected hazard poses to map coordinates
  - Publishes hazard signals to `/snc/hazard_signal` topic

### Node 3: Path Tracing (`path_tracing_node`)
- **Purpose**: Records and traces robot's path for return navigation
- **Responsibilities**:
  - Samples robot pose at regular intervals
  - Maintains history of visited locations (exploration breadcrumbs)
  - Calculates and publishes return trajectories
  - Handles triggering of return path when hazards are detected

### Node 4: Twist Multiplexer (`twist_mux`) - DISABLED
- **Purpose**: Central command velocity routing with safety controls for testing.
- **Note**: This node is currently disabled in this branch because it was not used in the demonstration, only development.
- **Responsibilities**:
  - Routes multiple command sources to single output
  - Blocks all commands during testing with testing_mode flag
  - Ensures safe testing without unintended robot movement

## ROSBot Pro 3 Specific Configuration

### Hardware Requirements
- ROSBot Pro 3 robot platform
- RGB camera (Oak-D series recommended)
- LiDAR sensor for navigation
- Properly calibrated sensors

### Software Dependencies
- ROS 2 Humble distribution
- Navigation 2 stack
- SLAM Toolbox 
- Find Object 2D for marker detection
- TF2 transformations

## Deployment Instructions

### Prerequisites
Before deploying the SNC package, ensure the following:

1. ROS 2 Humble is installed
2. Navigation 2 stack is installed
3. SLAM Toolbox is installed
4. Find Object 2D is installed
5. ROSBot Pro 3 drivers are properly configured
6. All required packages are built in your workspace

### Installation Steps

1. **Clone the repository**:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

2. **Install dependencies**:
```bash
rosdep install --from-paths . --ignore-src -y
```

3. **Build the package**:
```bash
colcon build --packages-select snc
source ~/ros2_ws/install/setup.bash
```

### Launch Procedures

#### Real Hardware Deployment

For deployment on real ROSBot Pro 3 hardware:
```bash
ros2 launch snc snc.launch.py
```

#### Simulation Environment

For Gazebo simulation:
```bash
ros2 launch snc sim.launch.py
```

#### Testing Configuration

For testing with command controls:
```bash
ros2 launch snc master.launch.py
```

## Configuration Management

### Parameter Tuning

All configurable parameters are defined in `config/params.yml`:

```yaml
/navigation_node:
  ros__parameters:
    planner_frequency: 1.0
    status_frequency: 1.0
    exploration_timeout_sec: 240.0
    spin_angular_speed: 0.8
    spin_angle_deg: 360.0
    min_frontier_cluster_size: 50
    frontier_standoff_m: 0.1

/path_tracing_node:
  ros__parameters:
    pose_sample_interval_s: 0.5
    waypoint_spacing_min: 0.35
    waypoint_rotation_min: 15.0

/marker_detection_node:
  ros__parameters:
    marker_confidence_threshold: 0.8
```

### Hazard Categories

The system supports 13 hazard categories:

| Category | ID | Description |
|----------|----|-------------|
| Unknown | 0 | Unidentified hazard |
| Explosive | 1 | Explosive materials |
| Flammable Gas | 2 | Gases that can ignite |
| Non-Flammable Gas | 3 | Non-flammable gases |
| Dangerous When Wet | 4 | Materials reactive with water |
| Flammable Solid | 5 | Solid flammable materials |
| Spontaneously Combustible | 6 | Self-igniting materials |
| Oxidizer | 7 | Oxidizing agents |
| Organic Peroxide | 8 | Organic peroxides |
| Inhalation Hazard | 9 | Toxic inhalation hazards |
| Poison | 10 | Poisonous substances |
| Radioactive | 11 | Radioactive materials |
| Corrosive | 12 | Corrosive substances |

## Operational Modes

### Demo Mode
- Simple configuration for demonstration purposes
- Focuses on core functionality without complex integrations
- Launch command: `ros2 launch snc snc.launch.py`

### Live Testing with Twist Mux
- Safe testing mode with command locking
- Prevents robot movement during testing phases
- Launch command: `ros2 launch snc master.launch.py`

### Integration with Mock Topics
- Full simulation environment ready for testing
- Uses mock topics to simulate real sensor data
- Launch command: `ros2 launch snc master.launch.py`

## Monitoring and Diagnostics

### Key Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/snc/hazard_signal` | `find_object_2d/ObjectsStamped` | Published when hazards are detected |
| `/snc/robot_pose` | `geometry_msgs/PoseStamped` | Current robot pose information |
| `/snc/return_waypoints` | `nav_msgs/Path` | Waypoints for return navigation |
| `/objectsStamped` | `find_object_2d/ObjectsStamped` | Hazard detection results |
| `/snc_start` | `std_msgs/Empty` | Start signal for the challenge |

### Diagnostic Commands

Monitor system status:
```bash
ros2 topic echo /snc/hazard_signal
ros2 topic echo /snc/robot_pose
```

## Troubleshooting

### Common Issues

1. **Nodes not starting**: Verify all dependencies are installed and workspace is sourced correctly
2. **No hazard detection**: Check camera feed and marker detection parameters
3. **Navigation failures**: Verify Nav2 configuration and map availability
4. **Sensor communication issues**: Confirm proper hardware connections and driver installation

### Logging

All nodes output logs to the console. Use the following to monitor:
```bash
ros2 node list
ros2 topic echo /snc/hazard_signal
ros2 topic echo /snc/robot_pose
```

## Performance Optimization

### Navigation Strategy
- Frontier exploration with wall-following fallback
- Minimum waypoint spacing to reduce path clutter
- Rotation threshold for smooth trajectory generation

### Memory Management
- Efficient breadcrumb storage with automatic cleanup
- Optimized path calculation using geometric thresholds
- Timed pose sampling to balance accuracy and resource usage

## Future Enhancements

1. **Advanced Exploration Algorithms**: Implementation of more sophisticated frontier-based exploration
2. **Multi-Robot Coordination**: Support for coordinated exploration by multiple robots
3. **Enhanced Hazard Classification**: Improved machine learning models for hazard detection
4. **Dynamic Path Replanning**: Real-time path adjustment based on changing conditions

## License

This package is licensed under the MIT License. See LICENSE for full details.
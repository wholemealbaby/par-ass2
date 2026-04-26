# SNC Package

ROS 2 package for autonomous navigation, hazard detection, and path tracing in search and clear operations.

## Overview

The SNC (Search and Clear) package provides a complete system for autonomous robot navigation with integrated hazard detection capabilities. It combines navigation, computer vision-based marker detection, and path tracing to enable robots to autonomously explore environments while detecting and classifying hazardous materials.

## Package Structure

```
snc/
├── launch/                    # Launch files
│   ├── snc.launch.py         # Main launch file for real hardware
│   ├── master.launch.py      # Master launch file combining components
│   └── find_object.launch.py # Find object 2D launch file
├── snc/                      # Python package
│   ├── __init__.py
│   ├── constants.py          # Constants and hazard mappings
│   ├── navigation_node.py    # Navigation control node
│   ├── marker_detection_node.py  # Hazard marker detection
│   ├── path_tracing_node.py  # Path recording and tracing
│   ├── twist_mux.py          # Twist multiplexer for command routing
│   ├── mock_topics.py        # Mock topics for testing
│   └── best_effort_repeater.py  # Repeater for QoS conversion
├── config/
│   └── params.yml            # Node parameters
├── test/                     # Test files
├── package.xml              # ROS 2 package definition
├── setup.py                 # Python package setup
└── setup.cfg
```

## Configuration Modes

### 1. Demo Mode

**Purpose**: 
A simplified configuration for demonstration purposes that removes complex integrations and focuses on core functionality.

**Nodes Included**:
- `navigation_node` - Main navigation controller for autonomous exploration
- `marker_detection_node` - Hazard marker detection using computer vision
- `path_tracing_node` - Records and traces robot's path for return navigation

**Configuration**:
- Includes twist_mux node configured for demo operations
- No mock topics publisher
- Direct integration with hardware or simulation

**Launch Command**:
```bash
ros2 launch snc snc.launch.py
```

### 2. Live Testing with Twist Mux

**Purpose**:
Configuration for live testing with command control locking. This mode prevents robot movements during testing phases.

**Nodes Included**:
- `navigation_node` - Main navigation controller
- `marker_detection_node` - Hazard marker detection
- `path_tracing_node` - Path recording and tracing
- `twist_mux` - Command multiplexer with testing mode lock

**Configuration**:
- Twist mux configured with testing mode enabled
- All movement commands blocked when testing_mode=True
- Used for safe testing of navigation and detection algorithms

**Launch Command**:
```bash
ros2 launch snc master.launch.py
```

### 3. Integration with Mock Topics

**Purpose**:
Configuration for integration testing using mock topics that simulate real sensor data without requiring actual hardware.

**Nodes Included**:
- `navigation_node` - Main navigation controller
- `marker_detection_node` - Hazard marker detection
- `path_tracing_node` - Path recording and tracing
- `mock_topics` - Mock topic publisher for testing
- `twist_mux` - Command multiplexer
- `find_object_2d` - Object detection system

**Configuration**:
- Includes mock topics publisher for simulated data
- Integration with find_object_2D for testing detection algorithms
- Full simulation environment ready for testing

**Launch Command**:
```bash
ros2 launch snc master.launch.py
```

## Nodes

### Navigation Node (`navigation_node`)
- **Purpose**: Main navigation controller for autonomous exploration
- **Functionality**: 
  - Coordinates robot movement through waypoints
  - Integrates with Nav2 for path planning and execution
  - Manages exploration strategies
- **Parameters** (configurable via `params.yml`):
  - `waypoint_spacing_min`: Minimum distance between waypoints (default: 0.15m)
  - `spin_duration_s`: Duration for rotation/spin maneuvers (default: 2.0s)

### Marker Detection Node (`marker_detection_node`)
- **Purpose**: Detects and classifies hazard markers using computer vision
- **Functionality**:
  - Processes camera feed to identify hazard markers
  - Classifies markers using predefined hazard categories
  - Publishes detection results to `/snc/hazard_signal` topic
- **Parameters**:
  - `marker_confidence_threshold`: Confidence threshold for detection (default: 0.8)

### Path Tracing Node (`path_tracing_node`)
- **Purpose**: Records and traces robot's path for return navigation
- **Functionality**:
  - Samples robot pose at regular intervals
  - Maintains history of visited locations
  - Provides waypoints for return path
- **Parameters**:
  - `pose_sample_interval_s`: Time interval for pose sampling (default: 0.5s)

### Twist Mux Node (`twist_mux`)
- **Purpose**: Central multiplexer for cmd_vel sources with testing mode lock
- **Functionality**:
  - Routes multiple Twist command sources to single output
  - Blocks all commands when testing_mode=True
  - Ensures safe testing without unintended robot movement
- **Input Topics**:
  - `/cmd_vel_nav` - Nav2 navigation source (remapped from /cmd_vel)
  - `/cmd_vel_raw` - Manual navigation commands (from NavigationNode)
  - `/cmd_vel_teleop` - Teleop keyboard/joystick input
  - `/cmd_vel_manual` - Manual override input
- **Output Topic**:
  - `/cmd_vel` - Only published when testing_mode=False (robot motor input)

## Hazard Classification

The package recognizes 13 hazard categories defined in `constants.py`:

| Hazard Type | ID | Description |
|-------------|----|-------------|
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

## Topics

- `/snc/hazard_signal`: Published when hazards are detected
- `/snc/robot_pose`: Current robot pose information  
- `/snc/return_waypoints`: Waypoints for return navigation
- `/objectsStamped`: Published by find_object_2d with hazard detection results
- `/snc_start`: Start signal for the challenge
- `/path_return`: Return path for navigation
- `/path_explore`: Exploration path for navigation

## Dependencies

### ROS 2 Packages
- `rclpy` - ROS 2 Python client library
- `nav_msgs`, `geometry_msgs`, `sensor_msgs` - ROS message types
- `nav2_msgs`, `action_msgs` - Navigation 2 messages
- `tf2_ros` - Transform library
- `cv_bridge` - OpenCV bridge
- `find_object_2d_msgs` - Object detection messages

### External Packages
- `aiil_gazebo` - Simulation environment
- `nav2_*` - Navigation 2 stack
- `slam_toolbox` - SLAM implementation
- `find_object_2d` - 2D object detection

## Installation

### Building from Source

1. Clone the package into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths . --ignore-src -y
   ```

3. Build the package:
   ```bash
   colcon build --packages-select snc
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Launching the Complete System

#### For Simulation (Gazebo):
```bash
ros2 launch snc sim.launch.py
```

#### For Real Hardware:
```bash
ros2 launch snc snc.launch.py
```

#### For Testing with Twist Mux:
```bash
ros2 launch snc master.launch.py
```

### Running Individual Nodes

You can also run nodes individually:

```bash
# Navigation node
ros2 run snc navigation_node

# Marker detection node  
ros2 run snc marker_detection_node

# Path tracing node
ros2 run snc path_tracing_node

# Twist mux node
ros2 run snc twist_mux

# Mock topics publisher
ros2 run snc mock_topics
```

### Configuration

Edit `config/params.yml` to adjust node parameters:

```yaml
/navigation_node:
  ros__parameters:
    waypoint_spacing_min: 0.15
    spin_duration_s: 2.0
            
/path_tracing_node:
  ros__parameters:
    pose_sample_interval_s: 0.5
    
/marker_detection_node:
  ros__parameters:
    marker_confidence_threshold: 0.8
```

## Testing

Run the package tests:

```bash
colcon test --packages-select snc
```

Available tests:
- `test_copyright.py` - Copyright header verification
- `test_flake8.py` - Python style checking
- `test_pep257.py` - Docstring style checking

## Development

### Adding New Hazard Types

1. Update `hazard_names_map` in `snc/constants.py`
2. Add corresponding marker images to the detection system
3. Update any classification logic in `marker_detection_node.py`

### Extending Functionality

- **New Navigation Behaviors**: Modify `navigation_node.py`
- **Additional Sensors**: Add new subscribers in relevant nodes
- **Custom Detection Algorithms**: Extend `marker_detection_node.py`

## Troubleshooting

### Common Issues

1. **Nodes not starting**: Ensure all dependencies are installed and the workspace is sourced
2. **No hazard detection**: Check camera feed and marker detection parameters
3. **Navigation failures**: Verify Nav2 configuration and map availability

### Logging

All nodes output logs to the console. Use `ros2 topic echo` to monitor topics:

```bash
ros2 topic echo /snc/hazard_signal
ros2 topic echo /snc/robot_pose
```

## License

TODO: Add license information (see `package.xml` for details)

## Maintainers

- tomg (tomgosling57@gmail.com)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with appropriate testing
4. Submit a pull request

## Acknowledgments

This package is part of the RMIT AI & Intelligent Systems Laboratory (AIIL) robotics platform.
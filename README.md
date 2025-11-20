# F1TENTH Autonomous Racing Platform

A comprehensive ROS2-based autonomous racing platform for the F1TENTH competition. This repository contains all the necessary packages for simulating, controlling, and navigating an F1TENTH race car with advanced mapping, localization, and autonomous navigation capabilities.

## Overview

This project implements a complete autonomous racing stack for F1TENTH vehicles, featuring:

- **Hardware Interface**: ROS2 Control integration with VESC motor controllers
- **Sensor Integration**: LiDAR (SICK), IMU, and camera support (Intel RealSense D435i)
- **SLAM & Mapping**: Real-time mapping using Cartographer and SLAM Toolbox
- **Localization**: AMCL-based localization with particle filtering
- **Navigation**: Nav2 stack with Regulated Pure Pursuit controller for Ackermann steering
- **Teleoperation**: Joy/keyboard control with safety features

## Repository Structure

```
F1tenth/
├── src/
│   ├── f1tenth_description/          # Robot description, URDF, launch files
│   │   ├── config/                   # Configuration files (Nav2, AMCL, sensors)
│   │   ├── launch/                   # Launch files for different modes
│   │   ├── maps/                     # Pre-built maps for navigation
│   │   ├── meshes/                   # 3D models and STL files
│   │   ├── urdf/                     # Robot URDF and Xacro files
│   │   └── src/                      # C++ source files
│   ├── f1tenth_hardware/             # ROS2 Control hardware interface
│   │   ├── config/                   # Hardware configuration
│   │   ├── include/                  # Header files
│   │   ├── src/                      # Hardware interface implementation
│   │   └── launch/                   # Hardware launch files
│   └── f1tenth_system/               # Core system drivers and utilities
├── frames_*.gv                       # TF tree visualization files
└── frames_*.pdf                      # TF tree PDF exports
```

## Dependencies

### System Requirements
- **OS**: Ubuntu 22.04 LTS (recommended)
- **ROS**: ROS2 Humble
- **Hardware**: F1TENTH car with VESC, LiDAR, IMU

### ROS2 Dependencies
```bash
# Core ROS2 packages
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher

# Navigation and SLAM
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

# Hardware and Control
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-hardware-interface

# Sensors
sudo apt install ros-humble-sick-scan-xd
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-joy
sudo apt install ros-humble-teleop-twist-joy

# VESC Driver
sudo apt install ros-humble-ackermann-msgs
```

## Quick Start

### 1. Clone and Build

```bash
# Create workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone repository
git clone https://github.com/pranavk-2003/F1tenth.git
cd F1tenth

# Clone submodules
git submodule update --init --recursive --remote

# Build the workspace
cd ~/f1tenth_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Hardware Setup

```bash
# Launch the complete F1TENTH system
ros2 launch f1tenth_description f1tenth.launch.py

# Or launch individual components:

# Hardware interface only
ros2 launch f1tenth_hardware f1tenth_ros2_control.launch.py

# Sensors only
ros2 launch f1tenth_description sensor_fusion.launch.py
```

### 3. SLAM and Mapping

```bash
# Start SLAM with Cartographer
ros2 launch f1tenth_description cartographer.launch.py

# Or use SLAM Toolbox
ros2 launch f1tenth_description slam.launch.py

# Save the map after mapping
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 4. Autonomous Navigation

```bash
# Launch navigation stack
ros2 launch f1tenth_description nav2.launch.py

# Use RViz to set navigation goals or send goals programmatically:
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

## Teleoperation

### Gamepad Control (Logitech F710)
- **LB Button**: Deadman's switch for manual teleop
- **RB Button**: Deadman's switch for navigation mode
- **Left Stick**: Steering control
- **Right Trigger**: Acceleration
- **Left Trigger**: Reverse

```bash
# Launch teleoperation
ros2 launch f1tenth_description f1tenth.launch.py
```

### Keyboard Control
```bash
# Install teleop-twist-keyboard if not available
sudo apt install ros-humble-teleop-twist-keyboard

# Launch keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Maps and Localization

### Pre-built Maps
The repository includes several pre-built maps in `src/f1tenth_description/maps/`:
- `slam_map.yaml/pgm`: Default lab map
- `canteen.yaml/pgm`: Canteen environment
- `labmap.yaml`: Laboratory environment

### AMCL Localization
```bash
# Launch localization with a specific map
ros2 launch f1tenth_description nav2.launch.py map:=/path/to/your/map.yaml

# Initialize pose estimate in RViz using "2D Pose Estimate" tool
```

## Configuration

### Key Configuration Files
- `config/nav2_params.yaml`: Navigation parameters
- `config/amcl.yaml`: Localization parameters  
- `config/vesc.yaml`: Motor controller settings
- `config/sensors.yaml`: Sensor configurations
- `config/mux.yaml`: Input multiplexer settings

### Tuning Navigation
Edit `config/nav2_params.yaml` to adjust:
- **Controller parameters**: Speed, steering limits
- **Path planning**: Global and local planner settings
- **Recovery behaviors**: Backup and rotation behaviors

## Hardware Interface

### VESC Configuration
The VESC motor controller interface supports:
- Speed control (rpm/velocity)
- Steering angle control
- Battery monitoring
- Temperature monitoring
- Fault detection

### Sensor Integration
- **LiDAR**: SICK TiM5xx series (configured for 270° scan)
- **Camera**: Intel RealSense D435i with depth
- **IMU**: Built-in IMU fusion for orientation
- **Encoders**: Wheel odometry through VESC

## Monitoring and Debugging

### Check System Status
```bash
# View active topics
ros2 topic list

# Monitor transforms
ros2 run tf2_tools view_frames
evince frames.pdf

# Check node graph
rqt_graph

# Monitor system performance
ros2 run rqt_robot_monitor rqt_robot_monitor
```

### Common Troubleshooting

1. **VESC Connection Issues**:
   ```bash
   # Check USB permissions
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

2. **LiDAR Not Found**:
   ```bash
   # Scan for LiDAR IP
   nmap -sn 192.168.1.0/24
   # Update IP in sensor configuration
   ```

3. **Transform Issues**:
   ```bash
   # Check TF tree
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo base_link laser
   ```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Documentation

- [F1TENTH Documentation](https://f1tenth.readthedocs.io/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Navigation Stack](https://navigation.ros.org/)
- [VESC Documentation](https://vesc-project.com/)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## F1TENTH Competition

This stack is designed for the [F1TENTH Autonomous Racing Competition](https://f1tenth.org/). Key features for competitive racing:

- **High-frequency control**: 100Hz control loop
- **Safety features**: Emergency stops and collision avoidance
- **Race-optimized parameters**: Aggressive but stable tuning
- **Telemetry**: Real-time performance monitoring

## Support

- **Issues**: Report bugs and feature requests via GitHub Issues
- **Discussions**: Join the F1TENTH community discussions
- **Wiki**: Check the project wiki for additional documentation

---

**Maintainer**: niat  
**Version**: 1.0.0  
**Last Updated**: August 2025

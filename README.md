# Autonomous mobile robot
A ROS-based autonomous mobile robot platform with 4-wheel differential drive configuration, equipped with LDS-02 2D LIDAR and RealSense D435i RGB-D camera for autonomous navigation and mapping.

Work in progress. To be updated.

## Features
- Autonomous Navigation
- SLAM Capability
- Obstacle Avoidance
- PS5 Controller Teleoperation
- Sensor Integration (LDS-02 LIDAR, RealSense D435i)
- EKF-based Localization

## Prerequisites

### ROS and System Dependencies
- Ubuntu 18.04
- ROS Melodic
- Gazebo 9
- Python 2.7

### Required ROS Packages
```bash
sudo apt-get install \
    ros-melodic-twist-mux \
    ros-melodic-robot-localization \
    ros-melodic-diff-drive-controller \
    ros-melodic-joint-state-controller \
    ros-melodic-velocity-controllers \
    ros-melodic-gazebo-ros-control \
    ros-melodic-joy \
    ros-melodic-teleop-twist-joy \
    ros-melodic-navigation
```

## Installation

1. Create a catkin workspace (if not already created):
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
```

2. Clone the repository:
```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/ros_mobile_robot.git
```

3. Build the workspace:
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## Usage (to be optimized)

### Launch the Robot in Simulation
```bash
roslaunch ros_mobile_robot mobile_robot_stage_4.launch
```
This launches:
- Gazebo simulation
- Robot controllers
- Navigation stack
- RViz visualization

### Manual Control with PS5 Controller
Ensure your PS5 controller is connected, then:
```bash
roslaunch ros_mobile_robot ps5_teleop.launch
```

### Autonomous Navigation
1. In RViz:
   - Use "2D Pose Estimate" to set initial robot position
   - Use "2D Nav Goal" to set navigation target
2. The robot will autonomously plan and execute a path to the goal

## System Architecture

### Hardware Configuration
- 4-wheel differential drive base
- LDS-02 2D LIDAR Scanner
- RealSense D435i RGB-D Camera with IMU
- PS5 Controller for manual operation

## Key Configuration Files
- `config/control.yaml`: Controller parameters
- `config/navigation.yaml`: Navigation stack parameters
- `config/localization.yaml`: EKF parameters
- `config/twist_mux.yaml`: Command multiplexing configuration

## To be Improved
- Sharp turns during obstacle avoidance need improvement
- Rotation movements could be more responsive
- Collision prevention needs enhancement

## Acknowledgments
- Built using ROS Melodic
- Uses components from the ROS Navigation Stack

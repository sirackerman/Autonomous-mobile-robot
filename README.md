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
git clone https://github.com/sirackerman/ros_mobile_robot.git
```

3. Build the workspace:
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## Usage

### Environment Setup
```bash
# Set the robot model
export MOBILE_ROBOT_MODEL=custom_robot
```
### Mapping with Gmapping
To create a map of a new environment:

1. Launch the desired Gazebo world:
```bash
roslaunch ros_mobile_robot mobile_robot_stage_1.launch  # For Stage 1
# Or other available worlds:
# mobile_robot_stage_2.launch
# mobile_robot_stage_3.launch
# mobile_robot_stage_4.launch
# house.launch
```
2. Launch the mapping system:
```bash
roslaunch ros_mobile_robot robot_mapping.launch
```

3. Control the robot to explore the environment:

With the keyboard:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
With the PS5 controller:

Install this [ROS_package](https://github.com/sirackerman/PS5-Controller-ROS-package), ensure your PS5 controller is connected, then run:
```bash
roslaunch ps5_teleop ps5_teleop.launch
```

### Autonomous Navigation
To navigate in a known environment using a previously created map:

1. Launch navigation with the desired world and map:
```bash
roslaunch ros_mobile_robot navigation.launch map_file:=/path/to/map.yaml world_file:=/path/to/world.world
```

Example:
```bash
roslaunch ros_mobile_robot navigation.launch map_file:=/home/user/catkin_ws/src/ros_mobile_robot/maps/map.yaml world_file:=/home/user/catkin_ws/src/ros_mobile_robot/worlds/stage_4.world
```

2. Set initial robot pose in RViz using "2D Pose Estimate"

3. Set navigation goals using "2D Nav Goal" in RViz

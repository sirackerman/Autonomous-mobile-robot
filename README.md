# Autonomous mobile robot
A ROS-based autonomous mobile robot platform with 4-wheel differential drive configuration, equipped with LDS-02 2D LIDAR and RealSense D435i RGB-D camera for mapping, autonomous exploration+mapping and autonomous navigation.

Work in progress. To be updated.

## Features
-Autonomous Navigation and obstacle avoidance
-Mapping
-Autonomous exploration and mapping

## Prerequisites

### ROS and System Dependencies
- Ubuntu 18.04 , ROS melodic or newer

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
To create a map of a new environment (uses turtlebot3 worlds):

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

4. After mapping the environment, save the map in your desired location:
```bash
# Save the map to your maps directory
rosrun map_server map_saver -f ~/catkin_ws/src/ros_mobile_robot/maps/my_map
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

### Autonomous Exploration

The robot supports autonomous exploration using RRT (Rapidly-exploring Random Tree) exploration strategy. This functionality combines SLAM for mapping with frontier-based exploration.

1. Robot in House Environment (launch file can be edited to spawn the robot in a different world)
```bash
roslaunch ros_mobile_robot robot_exploration.launch 
```

2. RRT Exploration
```bash
roslaunch ros_mobile_robot robot_rrt_exploration.launch
```
This initializes:
- SLAM (Gmapping)
- Global RRT detector
- Local RRT detector
- Frontier filter and assigner

#### Known Limitations and Future Work
   - Current implementation might need parameter tuning for better obstacle avoidance
   - Exploration strategy can be optimized for better coverage

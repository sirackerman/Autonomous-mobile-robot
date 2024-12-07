cmake_minimum_required(VERSION 3.0.2)
project(ros_mobile_robot)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        geometry_msgs
        tf
        pcl_ros
        pcl_conversions
        sensor_msgs
        move_base
        amcl
        message_generation
)

add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


find_package(PCL 1.8 REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf pcl_ros pcl_conversions sensor_msgs message_runtime
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_install_python(PROGRAMS
  scripts/functions.py
  scripts/global_rrt.py
  scripts/local_rrt.py
  scripts/filter.py
  scripts/assigner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# executable for point cloud to grid converter and exploration_node
add_executable(pointcloud_to_grid src/pointcloud_to_grid.cpp)
add_executable(exploration_node src/exploration_node.cpp)
add_executable(trajectory_follower src/trajectory_follower.cpp)
add_executable(tf_maintainer src/tf_maintainer.cpp)

target_link_libraries(tf_maintainer
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_follower
  ${catkin_LIBRARIES}
)


target_link_libraries(pointcloud_to_grid
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_link_libraries(exploration_node
        ${catkin_LIBRARIES}
)

#executables for installation
install(TARGETS pointcloud_to_grid exploration_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install launch and config files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        FILES_MATCHING PATTERN "*.yaml"
)

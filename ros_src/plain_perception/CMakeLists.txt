cmake_minimum_required(VERSION 3.0.2)
project(plain_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  urdf
  xacro
  message_generation
  std_msgs
  cv_bridge
)

add_action_files(
  DIRECTORY action
  FILES LandingDetection.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

catkin_package(

)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/LandingDetectionActionServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
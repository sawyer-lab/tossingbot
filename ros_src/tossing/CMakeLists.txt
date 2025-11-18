cmake_minimum_required(VERSION 3.0.2)
project(tossing)

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
)


add_action_files(
  DIRECTORY action
  FILES Toss.action
)
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tossing
#  CATKIN_DEPENDS gazebo_ros roscpp rospy urdf xacro
#  DEPENDS system_lib
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
  scripts/TossingActionServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
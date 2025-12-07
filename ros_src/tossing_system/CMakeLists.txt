cmake_minimum_required(VERSION 3.0.2)
project(tossingbot_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  intera_core_msgs
  tf2_ros
)

# This handles the Python module setup
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs intera_core_msgs
)

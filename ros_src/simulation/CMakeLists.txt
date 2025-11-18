cmake_minimum_required(VERSION 3.0.2)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  actionlib_msgs 
  message_generation
  std_msgs
  cv_bridge
)



generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
    geometry_msgs

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
   scripts/simulation.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

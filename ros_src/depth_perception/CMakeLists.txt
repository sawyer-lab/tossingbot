cmake_minimum_required(VERSION 3.0.2)
project(depth_perception)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  XYZRGBArray.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(voxel src/voxel.cpp)
target_link_libraries(voxel ${catkin_LIBRARIES})
add_dependencies(voxel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
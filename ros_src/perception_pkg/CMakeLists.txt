cmake_minimum_required(VERSION 3.0.2)
project(perception_pkg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  image_geometry
  pcl_ros
  pcl_conversions
)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport image_geometry pcl_ros pcl_conversions
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(depth_to_cloud_node src/depth_to_cloud_node.cpp)

target_link_libraries(depth_to_cloud_node
  ${catkin_LIBRARIES}
)


cmake_minimum_required(VERSION 3.0.2)
project(cinves_perception)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/keny2.py
  scripts/head_wobbler.py
  scripts/move_head.py
  scripts/camera_display.py
  scripts/fk.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


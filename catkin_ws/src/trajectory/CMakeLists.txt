cmake_minimum_required(VERSION 3.0.2)
project(trajectory)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark Python scripts for installation
catkin_install_python(PROGRAMS
  src/plan_traj.py
  src/follow_traj.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


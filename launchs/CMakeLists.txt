cmake_minimum_required(VERSION 2.8.3)
project(launchs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

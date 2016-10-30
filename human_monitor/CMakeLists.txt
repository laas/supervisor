cmake_minimum_required(VERSION 2.8.3)
project(human_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  supervisor_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs supervisor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(human_monitor src/main.cpp src/human_monitor.cpp)
target_link_libraries(
  human_monitor
  ${catkin_LIBRARIES}
)

add_dependencies(human_monitor supervisor_msgs_generate_messages_cpp)

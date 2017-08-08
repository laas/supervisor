cmake_minimum_required(VERSION 2.8.3)
project(human_simulator)

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

add_executable(human_simulator src/main.cpp)
target_link_libraries(
  human_simulator
  ${catkin_LIBRARIES}
)

add_dependencies(human_simulator supervisor_msgs_generate_messages_cpp)

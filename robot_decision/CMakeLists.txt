cmake_minimum_required(VERSION 2.8.3)
project(robot_decision)

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

add_executable(robot_decision src/main.cpp)
target_link_libraries(
  robot_decision
  ${catkin_LIBRARIES}
)

add_dependencies(robot_decision supervisor_msgs_generate_messages_cpp)

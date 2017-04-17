cmake_minimum_required(VERSION 2.8.3)
project(goal_manager)

find_package(catkin REQUIRED COMPONENTS
  toaster_msgs 
  message_generation
  roscpp
  rospy
  std_msgs
  supervisor_msgs
)

catkin_package(
   CATKIN_DEPENDS toaster_msgs roscpp rospy std_msgs supervisor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_manager src/main.cpp)
target_link_libraries( 
  goal_manager
  ${catkin_LIBRARIES}
)
add_dependencies(goal_manager supervisor_msgs_generate_messages_cpp)

cmake_minimum_required(VERSION 2.8.3)
project(mental_states)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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


add_executable(mental_states src/main.cpp src/ms_manager.cpp)
target_link_libraries(
  mental_states
  ${catkin_LIBRARIES}
)
add_dependencies(mental_states supervisor_msgs_generate_messages_cpp)


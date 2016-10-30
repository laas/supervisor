cmake_minimum_required(VERSION 2.8.3)
project(data_manager)

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

add_executable(data_manager src/main.cpp)
target_link_libraries(
  data_manager
  ${catkin_LIBRARIES}
)

add_dependencies(data_manager supervisor_msgs_generate_messages_cpp)

cmake_minimum_required(VERSION 2.8.3)
project(dialogue_node)

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
  $ENV{ROBOTPKG_BASE}/include
)


add_executable(dialogue_node src/main.cpp)
target_link_libraries(
  dialogue_node
  ${catkin_LIBRARIES}
)
add_dependencies(dialogue_node supervisor_msgs_generate_messages_cpp)

#Line to comment if you dont want dependencies to acapela
#add_definitions("-DACAPELA")

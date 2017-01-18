cmake_minimum_required(VERSION 2.8.3)
project(plan_maintainer)

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


add_executable(plan_maintainer src/main.cpp)
target_link_libraries(
  plan_maintainer
  ${catkin_LIBRARIES}
)
add_dependencies(plan_maintainer supervisor_msgs_generate_messages_cpp)


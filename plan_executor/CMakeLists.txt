cmake_minimum_required(VERSION 2.8.3)
project(plan_executor)

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

add_executable(plan_executor src/main.cpp)
target_link_libraries(
  plan_executor
  ${catkin_LIBRARIES}
)

add_dependencies(plan_executor supervisor_msgs_generate_messages_cpp)

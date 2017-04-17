cmake_minimum_required(VERSION 2.8.3)
project(plan_elaboration)

find_package(catkin REQUIRED COMPONENTS
  toaster_msgs 
  message_generation
  roscpp
  rospy
  std_msgs
  supervisor_msgs
  hatp_msgs
)

catkin_package(
   CATKIN_DEPENDS toaster_msgs roscpp rospy std_msgs supervisor_msgs hatp_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(plan_elaboration src/main.cpp src/plan_elaboration.cpp src/virtual_domain.cpp src/Domains/default_domain.cpp src/Domains/scan_domain.cpp src/Domains/blocks_domain.cpp src/Domains/scan_us_domain.cpp)
target_link_libraries( 
  plan_elaboration
  ${catkin_LIBRARIES}
)
add_dependencies(plan_elaboration supervisor_msgs_generate_messages_cpp)

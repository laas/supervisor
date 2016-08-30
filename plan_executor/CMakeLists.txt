cmake_minimum_required(VERSION 2.8.3)
project(plan_executor)

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
#  INCLUDE_DIRS include
#  LIBRARIES mental_states
   CATKIN_DEPENDS toaster_msgs roscpp rospy std_msgs supervisor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{TOASTER_DIR}/devel/include
  $ENV{ROBOTPKG_BASE}/include
)


add_executable(plan_executor src/main.cpp)
target_link_libraries(
  plan_executor 
  ${catkin_LIBRARIES}
)
add_dependencies(plan_executor supervisor_msgs_generate_messages_cpp)


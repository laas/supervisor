cmake_minimum_required(VERSION 2.8.3)
project(action_executor)

find_package(PkgConfig)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  supervisor_msgs
  actionlib
  actionlib_msgs
)

pkg_check_modules(pr2motion-genom3 REQUIRED pr2motion-genom3)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs supervisor_msgs actionlib actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pr2motion-genom3_INCLUDE_DIRS}
  $ENV{ROBOTPKG_BASE}/include
)

link_directories(${pr2motion-genom3_LIBRARY_DIRS})

add_executable(action_executor src/main.cpp src/action_executor.cpp src/virtual_action.cpp src/Actions/pick.cpp src/Actions/place.cpp)
target_link_libraries(
  action_executor
  ${catkin_LIBRARIES}
)

add_dependencies(action_executor supervisor_msgs_generate_messages_cpp)

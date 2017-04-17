cmake_minimum_required(VERSION 2.8.3)
project(tests)

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
  $ENV{ROBOTPKG_BASE}/include
)

add_executable(test_pickandplace src/test_pickandplace.cpp)
target_link_libraries(
  test_pickandplace
  ${catkin_LIBRARIES}
)

add_dependencies(test_pickandplace supervisor_msgs_generate_messages_cpp)

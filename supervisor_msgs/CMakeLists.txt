cmake_minimum_required(VERSION 2.8.3)
project(supervisor_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  toaster_msgs
)

add_message_files(
   FILES
    Action.msg
    ActionsList.msg
    GoalsList.msg
    Goal.msg
    SharedPlan.msg
    Link.msg
    MentalState.msg
    MentalStatesList.msg
    Info.msg
)

add_service_files(
  FILES
    HumanAction.srv
    String.srv
    EndPlan.srv
    Ask.srv
    GiveInfo.srv
)

 add_action_files(
   FILES
    ActionExecutor.action
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   toaster_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_generation message_runtime actionlib actionlib_msgs toaster_msgs
)

include_directories(
  include
  $ENV{TOASTER_DIR}/devel/include
)



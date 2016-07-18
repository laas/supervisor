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


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   Action.msg
   Plan.msg
   ActionMS.msg
   PlanMS.msg
   GoalMS.msg
   Link.msg
   AgentActivity.msg
   Focus.msg
   Bool.msg
   AgentList.msg
<<<<<<< HEAD
=======
   AgentKnowledge.msg
   Knowledge.msg
   ActionMSList.msg
>>>>>>> e0a6ca3... add missing line in CMakeList
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  NewGoal.srv
  InfoGiven.srv
  HumanActionSimu.srv
  SolveDivergentBelief.srv
  EndPlan.srv
  ChangeState.srv
  GetInfo.srv
  Empty.srv
  HumanAction.srv
  Say.srv
  GiveInfo.srv
  Ask.srv
  GetInfoDia.srv
)

 add_action_files(
   FILES
   ActionExecutor.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   toaster_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_generation message_runtime actionlib actionlib_msgs toaster_msgs
#  DEPENDS system_lib
)

include_directories(
  include
#${catkin_INCLUDE_DIRS}
  $ENV{TOASTER_DIR}/devel/include
)



#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gtp_ros_msg/requestAction.h>
#include <gtp_ros_msg/Req.h>
#include <gtp_ros_msg/Ag.h>
#include <gtp_ros_msg/Obj.h>
#include <gtp_ros_msg/Iden.h>
#include <pr2motion/Arm_Right_MoveAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_pick");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<gtp_ros_msg::requestAction> ac("gtp_ros_server", true);
  actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction > ac_move("pr2motion/Arm_Right_Move",true);

  ROS_INFO("Waiting for GTP and pr2motion action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ac_move.waitForServer(); //will wait for infinite time

  ROS_INFO("GTP and pr2motion Action server started, sending goal.");
  // send a goal to the action
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "update";
  ac.sendGoal(goal);

  //wait for the action to return
  bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

  if (finishedBeforeTimeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");



  goal.req.requestType = "planning";
  goal.req.actionName = "pick";
  gtp_ros_msg::Ag agent;
  agent.actionKey = "mainAgent";
  agent.agentName = "PR2_ROBOT";
  goal.req.involvedAgents.push_back(agent);
  gtp_ros_msg::Obj object;
  object.actionKey = "mainObject";
  object.objectName = "RED_CUBE";
  goal.req.involvedObjects.push_back(object);
  goal.req.predecessorId.actionId = -1;
  goal.req.predecessorId.alternativeId = -1;
   ac.sendGoal(goal);

  //wait for the action to return
  finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

  if (finishedBeforeTimeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");


  goal.req.requestType = "load";
  goal.req.loadAction = ac.getResult()->ans.identifier;
  goal.req.loadSubTraj = 0;
 ac.sendGoal(goal);

  //wait for the action to return
  finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

  if (finishedBeforeTimeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

   ROS_INFO("Action server started, sending goal.");
	
  pr2motion::Arm_Right_MoveGoal arm_goal;
  arm_goal.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
  arm_goal.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
  ac_move.sendGoal(arm_goal);
  finishedBeforeTimeout = ac_move.waitForResult(ros::Duration(5.0));
  if(finishedBeforeTimeout)
    {
      actionlib::SimpleClientGoalState state = ac_move.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
  else
    ROS_INFO("Action did not finish before the time out\n");

  //exit
  return 0;
}

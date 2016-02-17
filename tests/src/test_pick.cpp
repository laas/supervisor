#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gtp_ros_msg/requestAction.h>
#include <gtp_ros_msg/Req.h>
#include <gtp_ros_msg/Ag.h>
#include <gtp_ros_msg/Obj.h>
#include <gtp_ros_msg/Iden.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_pick");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<gtp_ros_msg::requestAction> ac("gtp_ros_server", true);

  ROS_INFO("Waiting for GTP action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("GTP Action server started, sending goal.");
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
 // ac.sendGoal(goal);

  //wait for the action to return
  //finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

  if (finishedBeforeTimeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");


   


  //exit
  return 0;
}

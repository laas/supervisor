#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Torso_MoveAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "pr2motion_test");

  // create the action client
  // true causes the client to spin its own thread
  ros::NodeHandle n;
  actionlib::SimpleActionClient<pr2motion::InitAction> init("pr2motion/Init", true);
  actionlib::SimpleActionClient<pr2motion::Torso_MoveAction> ac("pr2motion/Torso_Move", true);
  ros::ServiceClient connect = n.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");

  ROS_INFO("Waiting for pr2motion action server to start.");
  // wait for the action server to start
  init.waitForServer(); //will wait for infinite time
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("pr2motion Action server started, sending goal.");
  
  //init
  pr2motion::InitGoal goal_init;
  init.sendGoal(goal_init);
  
  pr2motion::connect_port srv;
  srv.request.local = "joint_state";
  srv.request.remote = "joint_states";
  if (!connect.call(srv)){
    ROS_ERROR("[mental_state] Failed to call service pr2motion/connect_port");
  }
  srv.request.local = "head_controller_state";
  srv.request.remote = "/head_traj_controller/state";
  if (!connect.call(srv)){
    ROS_ERROR("[mental_state] Failed to call service pr2motion/connect_port");
  }
  srv.request.local = "traj";
  srv.request.remote = "gtp_trajectory";
  if (!connect.call(srv)){
    ROS_ERROR("[mental_state] Failed to call service pr2motion/connect_port");
  }
  
  // send a goal to the action
  pr2motion::Torso_MoveGoal goal;
  goal.torso_position = 0.2;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(300.0));

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

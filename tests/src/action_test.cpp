#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <supervisor_msgs/ActionExecutorAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> ac("supervisor/action_executor", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  supervisor_msgs::ActionExecutorGoal goal;
  goal.action.name = "pick";
  goal.action.id = 0;
  goal.action.parameters.push_back("red_cube");
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

  //exit
  return 0;
}

#include <ros/ros.h>
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/FactList.h"
#include "supervisor_msgs/NewPlan.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/StartGoal.h"
#include "supervisor_msgs/Link.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/Plan.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

  ros::NodeHandle n;
  ros::ServiceClient new_goal = n.serviceClient<supervisor_msgs::NewGoal>("mental_state/new_goal");
  ros::ServiceClient start_goal = n.serviceClient<supervisor_msgs::StartGoal>("mental_state/start_goal");
  ros::ServiceClient new_plan = n.serviceClient<supervisor_msgs::NewPlan>("mental_state/new_plan");
  
  supervisor_msgs::NewGoal srv_ngoal;
  srv_ngoal.request.goal = "CLEAN";

  if (new_goal.call(srv_ngoal))
  {
    ROS_INFO("Goal CLEAN added");
  }
  else
  {
    ROS_ERROR("Failed to call service mental_state/new_goal");
    return 1;
  }

  supervisor_msgs::StartGoal srv_sgoal;
  srv_sgoal.request.goal = "CLEAN";

  if (start_goal.call(srv_sgoal))
  {
    ROS_INFO("Goal CLEAN started");
  }
  else
  {
    ROS_ERROR("Failed to call service mental_state/start_goal");
    return 1;
  }

  supervisor_msgs::NewPlan srv_plan;
  supervisor_msgs::Plan plan;
  supervisor_msgs::Link link;
  supervisor_msgs::Action action1, action2;
  plan.goal = "CLEAN";
  action1.name = "pick";
  action1.id = 0;
  action1.parameters.push_back("RED_CUBE");
  action1.actors.push_back("PR2_ROBOT");
  plan.actions.push_back(action1);
  action2.name = "place";
  action2.id = 1;
  action2.parameters.push_back("RED_CUBE");
  action2.parameters.push_back("TABLE_4");
  action2.actors.push_back("PR2_ROBOT");
  plan.actions.push_back(action2);
  link.origin = 0;
  link.following = 1;
  plan.links.push_back(link);
  srv_plan.request.plan = plan;
  if (new_plan.call(srv_plan))
  {
    ROS_INFO("Plan started");
  }
  else
  {
    ROS_ERROR("Failed to call service mental_state/new_plan");
    return 1;
  }

  //exit
  return 0;
}

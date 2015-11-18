#include <ros/ros.h>
#include <vector>
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/AddFactsToAgent.h"
#include "supervisor_msgs/NewPlan.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/StartGoal.h"
#include "supervisor_msgs/SharePlan.h"
#include "supervisor_msgs/Link.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/Plan.h"

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

  ros::NodeHandle n;
  ros::ServiceClient new_goal = n.serviceClient<supervisor_msgs::NewGoal>("mental_state/new_goal");
  ros::ServiceClient start_goal = n.serviceClient<supervisor_msgs::StartGoal>("mental_state/start_goal");
  ros::ServiceClient new_plan = n.serviceClient<supervisor_msgs::NewPlan>("mental_state/new_plan");
  ros::ServiceClient add_fact = n.serviceClient<toaster_msgs::AddFactsToAgent>("database/add_facts_to_agent");
  ros::ServiceClient share_plan = n.serviceClient<supervisor_msgs::SharePlan>("mental_state/share_plan");


  vector<toaster_msgs::Fact> to_add;
  
  toaster_msgs::Fact fact;
  fact.subjectId = "RED_CUBE";
  fact.property = "isReachableBy";
  fact.targetId = "PR2_ROBOT";
  to_add.push_back(fact);

  toaster_msgs::Fact fact1;
  fact1.subjectId = "HERAKLES_HUMAN1";
  fact1.property = "isVisibleBy";
  fact1.targetId = "PR2_ROBOT";
  to_add.push_back(fact1);

  toaster_msgs::Fact fact11;
  fact11.subjectId = "PR2_ROBOT";
  fact11.property = "isVisibleBy";
  fact11.targetId = "HERAKLES_HUMAN1";
  to_add.push_back(fact11);

  toaster_msgs::Fact fact2;
  fact2.subjectId = "TABLE_4";
  fact2.property = "isReachableBy";
  fact2.targetId = "PR2_ROBOT";
  to_add.push_back(fact2);

  toaster_msgs::AddFactsToAgent srvAdd;
  srvAdd.request.agentId = "PR2_ROBOT";
  srvAdd.request.facts = to_add;
  if (!add_fact.call(srvAdd)){
    ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  }

  srvAdd.request.agentId = "HERAKLES_HUMAN1";
  srvAdd.request.facts = to_add;
  if (!add_fact.call(srvAdd)){
    ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  }

  supervisor_msgs::NewGoal srv_ngoal;
  srv_ngoal.request.goal = "TEST";

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
  srv_sgoal.request.goal = "TEST";

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


 supervisor_msgs::SharePlan srv_splan;

  if (share_plan.call(srv_splan))
  {
    ROS_INFO("Plan Shared");
  }
  else
  {
    ROS_ERROR("Failed to call service mental_state/share_plan");
    return 1;
  }


   ros::Duration(2).sleep();

  to_add = vector<toaster_msgs::Fact>();
 

  toaster_msgs::Fact fact3;
  fact3.subjectId = "RED_CUBE";
  fact3.property = "isOn";
  fact3.targetId = "TABLE_4";
  to_add.push_back(fact3);

  srvAdd.request.agentId = "PR2_ROBOT";
  srvAdd.request.facts = to_add;
  if (!add_fact.call(srvAdd)){
    ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  }
  

  //exit
  return 0;
}

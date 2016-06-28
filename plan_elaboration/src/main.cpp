/**
author Sandra Devin

Main class of the goal_manager.

The plan elaboration allows to find a plan to execute a goal

**/

#include "plan_elaboration/plan_elaboration.h"


ros::NodeHandle* node;
PlanElaboration* pe;

/*
Service call to execute a new goal
*/
bool newGoal(supervisor_msgs::NewGoal::Request  &req, supervisor_msgs::NewGoal::Response &res){

    pe->setGoal(req.goal);

    return true;
}

/*
Service call when the current plan is over
*/
bool endPlan(supervisor_msgs::EndPlan::Request  &req, supervisor_msgs::EndPlan::Response &res){

    if(req.objectLocked.size()){
        pe->dom_->objectLocked_ = req.objectLocked;
        pe->dom_->agentLocked_ = req.agentLocked;
    }
    pe->endPlan(req.report);

    return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "plan_elaboration");
  ros::NodeHandle _node;
  node = &_node;
  ros::Rate loop_rate(30);

  ROS_INFO("[plan_elaboration] Init plan_elaboration");

  PlanElaboration _pe(node);
  pe = &_pe;

  ros::ServiceServer service_goal = node->advertiseService("plan_elaboration/new_goal", newGoal); //new goal to execute
  ros::ServiceServer service_end = node->advertiseService("plan_elaboration/endPlan", endPlan); //new goal to execute

  ros::Publisher partners_pub = node->advertise<supervisor_msgs::AgentList>("/plan_elaboration/partners", 1000);

  ROS_INFO("[plan_elaboration] Plan_elaboration ready");

  while(_node.ok()){
    pe->checkPlan();
    supervisor_msgs::AgentList msg;
    msg.agents.push_back(pe->goalPartner_);
    partners_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  ros::spin();

  return 0;
}

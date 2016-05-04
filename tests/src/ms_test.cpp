#include <ros/ros.h>
#include <vector>
#include <ctime>
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/SetInfoDB.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/GetInfo.h"
#include "supervisor_msgs/Link.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/Plan.h"

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_action");

  ros::NodeHandle n;
  ros::ServiceClient add_fact = n.serviceClient<toaster_msgs::SetInfoDB>("database/set_info");
  ros::ServiceClient change_state = n.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");


    supervisor_msgs::ChangeState srv;
	
	supervisor_msgs::Action action;
   action.name = "pick";
   action.id = 0;
   action.parameters.push_back("RED_CUBE");
   action.actors.push_back("PR2_ROBOT");
   srv.request.type = "action";
	srv.request.action = action;
   srv.request.state = "PROGRESS";
   
   double tbegin,tend;
   tbegin=clock();
   
   ROS_ERROR("[action_executor] Before exec");
    if (!change_state.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
	}
    tend=clock();
    double texec=(tend - tbegin)/(CLOCKS_PER_SEC) * 1000;
   ROS_ERROR("[action_executor] Exec time: %f", texec);

  vector<toaster_msgs::Fact> toAdd;
  
  toaster_msgs::Fact fact;
  fact.subjectId = "RED_CUBE";
  fact.property = "isReachableBy";
  fact.targetId = "PR2_ROBOT";
  toAdd.push_back(fact);

  toaster_msgs::Fact fact1;
  fact1.subjectId = "HERAKLES_HUMAN1";
  fact1.property = "isVisibleBy";
  fact1.targetId = "PR2_ROBOT";
  toAdd.push_back(fact1);

  toaster_msgs::Fact fact11;
  fact11.subjectId = "PR2_ROBOT";
  fact11.property = "isVisibleBy";
  fact11.targetId = "HERAKLES_HUMAN1";
  toAdd.push_back(fact11);

  toaster_msgs::Fact fact2;
  fact2.subjectId = "TABLE_4";
  fact2.property = "isReachableBy";
  fact2.targetId = "PR2_ROBOT";
  toAdd.push_back(fact2);

  toaster_msgs::SetInfoDB srvAdd;
  srvAdd.request.infoType = "FACT";
  srvAdd.request.add = true;
  srvAdd.request.agentId = "PR2_ROBOT";
  srvAdd.request.facts = toAdd;
  //if (!add_fact.call(srvAdd)){
  //  ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  //}

  srvAdd.request.agentId = "HERAKLES_HUMAN1";
  srvAdd.request.facts = toAdd;
  //if (!add_fact.call(srvAdd)){
  //  ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  //}

  srv.request.type = "goal";
  srv.request.state = "NEW";
  srv.request.goal = "TEST";

  //if (change_state.call(srv))
  //{
  //  ROS_INFO("Goal CLEAN added");
  //}
  //else
  //{
  //  ROS_ERROR("Failed to call service mental_state/change_state");
  //  return 1;
  //}

  srv.request.type = "goal";
  srv.request.state = "PROGRESS";
  srv.request.goal = "TEST";

  //if (change_state.call(srv))
  //{
  //  ROS_INFO("Goal CLEAN started");
  //}
  //else
  //{
  //  ROS_ERROR("Failed to call service mental_state/change_state");
  //  return 1;
  //}

  supervisor_msgs::Plan plan;
  supervisor_msgs::Link link;
  supervisor_msgs::Action action1, action2;
  plan.goal = "CLEAN";
  plan.id = 0;
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
  srv.request.type = "plan";
  srv.request.state = "PROGRESS";
  srv.request.plan = plan;
  //if (change_state.call(srv))
  //{
  //  ROS_INFO("Plan started");
  //}
  //else
  //{
  //  ROS_ERROR("Failed to call service mental_state/change_state");
  //  return 1;
  //}


  srv.request.type = "plan";
  srv.request.state = "SHARE";

  //if (change_state.call(srv))
  //{
   // ROS_INFO("Plan Shared");
  //}
  //else
  //{
  //  ROS_ERROR("Failed to call service mental_state/change_state");
  //  return 1;
  //}


   //ros::Duration(2).sleep();


  srv.request.type = "action";
 srv.request.action = action1;
 srv.request.state = "DONE";
 // if (change_state.call(srv))
 // {
 //   ROS_INFO("Action 0 DONE");
 // }
 // else
 // {
 //   ROS_ERROR("Failed to call service mental_state/change_state");
 //   return 1;
 // }


  toAdd = vector<toaster_msgs::Fact>();
 

  toaster_msgs::Fact fact3;
  fact3.subjectId = "RED_CUBE";
  fact3.property = "isOn";
  fact3.targetId = "TABLE_4";
  toAdd.push_back(fact3);

  srvAdd.request.agentId = "PR2_ROBOT";
  srvAdd.request.facts = toAdd;
  //if (!add_fact.call(srvAdd)){
  //  ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
 // }
  

  //exit
  return 0;
}

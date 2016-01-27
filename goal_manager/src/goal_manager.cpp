/**
author Sandra Devin

**/

#include <goal_manager/goal_manager.h>


GoalManager::GoalManager(){
   currentGoal_ = "NONE";
}

/*
New goal to manage
*/
void GoalManager::addGoal(string goal){

   ros::NodeHandle node;
   ros::ServiceClient client = node.serviceClient<supervisor_msgs::NewGoal>("mental_state/start_goal");

   if(currentGoal_ == "NONE"){//if there is no current goal, we execute the goal
      //We say to the mental state manager that we start the goal
      supervisor_msgs::NewGoal srv;
	   srv.request.goal = goal;
	   if (!client.call(srv)){
	   ROS_ERROR("[goal_manager] Failed to call service mental_state/start_goal");
	   }
	   //And we execute it
      currentGoal_ = goal;
      ROS_INFO("[goal_manager] Executing the goal %s", goal.c_str());
      executeGoal(goal);
   }else{//else we put it in the waiting queue
      waitingGoals_.push(goal);
   }
}

/*
Executes a goal: compute facts, look for a plan and execute it if found, else failed the goal.
*/
void GoalManager::executeGoal(string goal){

   ros::NodeHandle node;
   ros::ServiceClient client = node.serviceClient<hatp_msgs::PlanningRequest>("Planner");
   ros::ServiceClient clientNP = node.serviceClient<supervisor_msgs::NewPlan>("mental_state/new_plan");
   ros::ServiceClient clientSP = node.serviceClient<supervisor_msgs::SharePlan>("mental_state/share_plan");
	hatp_msgs::PlanningRequest service;
	supervisor_msgs::NewPlan serviceNP;
	supervisor_msgs::SharePlan serviceSP;
	
	//We look for the HATP method name to call
	string methodTopic = "HATP_domains/";
	methodTopic = methodTopic + goal + "_method";
	string methodName;
	node.getParam(methodTopic, methodName);
	service.request.request.task=methodName;
	
	//We look for the HATP parameters to add
	string paramsTopic = "HATP_domains/";
	paramsTopic = paramsTopic + goal + "_params";
	vector<string> params;
	node.getParam(paramsTopic, params);
	for(vector<string>::iterator it = params.begin(); it != params.end(); it++){
	   service.request.request.parameters.push_back(*it);
	}
	
	//We ask a plan to HATP
	service.request.request.type="all";
	if(client.call(service)){
	   //we convert the plan in the supervisor format
	   supervisor_msgs::Plan newPlan = convertPlan(service.response.solution, goal);
	   //we send him to the mental state manager
	   serviceNP.request.plan = newPlan;
	   if(!clientNP.call(serviceNP)){
		   ROS_ERROR("Failed to call service mental_state/new_plan");
	   }
	   //For now, we consider the plan automatically shared
	   if(!clientSP.call(serviceSP)){
		   ROS_ERROR("Failed to call service mental_state/share_plan");
	   }
	}else{
		ROS_ERROR("Failed to call service 'Planner'");
	}
}

/*
Function which convert a plan from HATP to a supervisor plan
*/
supervisor_msgs::Plan GoalManager::convertPlan(hatp_msgs::Plan plan, string goal){

   ros::NodeHandle node;
   supervisor_msgs::Plan newPlan;
   newPlan.goal = goal;
   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
	      supervisor_msgs::Action action;
	      string nameTopic = "HATP_actions/";
	      nameTopic = nameTopic + it->name;
	      node.getParam(nameTopic, action.name);
	      action.id = it->id;
	      action.actors = it->agents;
	      //we remove the first parameter as it is the name of the agent
	      it->parameters.erase(it->parameters.begin());
	      action.parameters = it->parameters;
	      newPlan.actions.push_back(action);
	   }
	}
	for(vector<hatp_msgs::StreamNode>::iterator it = plan.streams.begin(); it != plan.streams.end(); it++){
	   for(vector<unsigned int>::iterator itt = it->successors.begin(); itt != it->successors.end(); itt++){
	      supervisor_msgs::Link link;
	      link.origin = it->taskId;
	      link.following = *itt;
	      newPlan.links.push_back(link);
	   }
	}
	
	return newPlan;
}
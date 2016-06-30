/**
author Sandra Devin

**/

#include <goal_manager/goal_manager.h>


GoalManager::GoalManager(){
   currentGoal_ = "NONE";
   hasPlan_ = false;
}

/*
New goal to manage
*/
void GoalManager::addGoal(string goal){
   
      waitingGoals_.push(goal);
}
/*
When a goal is over, look if there is another goal to execute
*/
void GoalManager::chooseGoal(){

   ros::NodeHandle node;
   ros::ServiceClient client = node.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");

   if(currentGoal_ != "NONE" && !hasPlan_){
      executeGoal(currentGoal_);
   }else if(currentGoal_ == "NONE" && waitingGoals_.size() >0){//there at least one waiting goal and no current goal
      string newGoal = waitingGoals_.front();
      waitingGoals_.pop();
      supervisor_msgs::ChangeState srv;
      srv.request.type = "goal";
      srv.request.state = "PROGRESS";
	   srv.request.goal = newGoal;
	   if (!client.call(srv)){
       ROS_ERROR("[goal_manager] Failed to call service mental_state/change_state");
	   }
	   //And we execute it
      currentGoal_ = newGoal;
      ROS_INFO("[goal_manager] Executing the goal %s", newGoal.c_str());
      executeGoal(newGoal);
   }
}

/*
Executes a goal: look for a plan and execute it if found, else failed the goal.
*/
void GoalManager::executeGoal(string goal){

   ros::NodeHandle node;
   ros::ServiceClient client = node.serviceClient<hatp_msgs::PlanningRequest>("Planner");
   ros::ServiceClient clientCS = node.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
	hatp_msgs::PlanningRequest service;
    supervisor_msgs::ChangeState serviceCS;
	
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
	service.request.request.type="plan";
	if(client.call(service)){
	   //we convert the plan in the supervisor format
	   if(service.response.solution.report == "OK"){
	      hasPlan_ = true;
	      supervisor_msgs::Plan newPlan = convertPlan(service.response.solution, goal);
          //we send him to the mental state manager
          //For now, we consider the plan automatically shared
          serviceCS.request.type = "plan";
          serviceCS.request.state = "PROGRESS_SHARE";
          serviceCS.request.plan = newPlan;
          if(!clientCS.call(serviceCS)){
              ROS_ERROR("Failed to call service mental_state/change_state");
          }
	   }else{//the robot aborts the goal
          ROS_INFO("[goal_manager] Aborting the goal %s", goal.c_str());
	      string robotName;
          node.getParam("/robot/name", robotName);
          serviceCS.request.type = "goal";
          serviceCS.request.state = "ABORT";
          serviceCS.request.agent = robotName;
          serviceCS.request.goal = goal;
          currentGoal_ = "NONE";
          if(!clientCS.call(serviceCS)){
              ROS_ERROR("Failed to call service mental_state/change_state");
          }
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

/*
Function call when the plan ends
*/
void GoalManager::endPlan(bool report){

   hasPlan_ = false;
   if(report){//the goal is achieved too, no need to get a new plan
      currentGoal_ = "NONE";
   }
}

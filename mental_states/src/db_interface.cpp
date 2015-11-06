/**
author Sandra Devin

Allows to make the link with the database of toaster

**/

#include <mental_states/db_interface.h>

/*
Return all the agents present in the database
*/
vector<string> DBInterface::getAgents(){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::GetAgents>("database/get_agents");
	toaster_msgs::GetAgents srv;
	vector<string> agents;
  	if (client.call(srv))
  	{
	  //return only the name of the agent
	  vector<toaster_msgs::Id> res = srv.response.resId;
	  for(vector<toaster_msgs::Id>::iterator it = res.begin(); it != res.end(); it++){
		agents.push_back(it->name);
 	  }
  	}
  	else
  	{
   	 ROS_ERROR("[mental_state] Failed to call service database/get_agents");
  	}
  	return agents;
}


/*
Function which add the state of a goal in an agent knowledge
	@goal: the goal
	@agent: the agent name 
	@goal: the state we want to put
*/
void DBInterface::addGoalState(supervisor_msgs::GoalMS goal, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> to_remove;

	//we remove previous state
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	toaster_msgs::Fact fact_rm;
	fact_rm.subjectId = goal.name;
	fact_rm.property = "goalState";
	to_remove.push_back(fact_rm);
	srv_rm.request.facts = to_remove;
	if (!client_rm.call(srv_rm)){
	 ROS_ERROR("Failed to call service database/remove_fact_to_agent");
	}
	
	//Then we add the new state
	toaster_msgs::AddFactToAgent srv;
	srv.request.agentId = agent;
	toaster_msgs::Fact fact;
	fact.subjectId = goal.name;
	fact.property = "goalState";
	fact.targetId = state;
	srv.request.fact = fact;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_fact_to_agent");
  	}
}

/*
Function which add the state of a plan in an agent knowledge
	@goal: the plan
	@agent: the agent name 
	@goal: the state we want to put
*/
void DBInterface::addPlanState(supervisor_msgs::PlanMS plan, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> to_remove;

	//retrieve previous state of the plan
	toaster_msgs::ExecuteSQL srvSQL;
	ostringstream to_string;
        to_string << plan.id;
        string plan_id = to_string.str();
	//we remove previous state
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	toaster_msgs::Fact fact_rm;
	fact_rm.subjectId = plan_id;
	fact_rm.property = "planState";
	to_remove.push_back(fact_rm);
	srv_rm.request.facts = to_remove;
	if (!client_rm.call(srv_rm)){
	 ROS_ERROR("[mental_state] Failed to call service database/remove_fact_to_agent");
	}
	//then we add the new state
	toaster_msgs::AddFactToAgent srv;
	srv.request.agentId = agent;
	toaster_msgs::Fact fact;
	fact.subjectId = plan_id;
	fact.property = "planState";
	fact.targetId = state;
	srv.request.fact = fact;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_fact_to_agent");
  	}
}

/*
Function which add the state of an action in an agent knowledge
	@goal: the action
	@agent: the agent name 
	@goal: the state we want to put
*/
void DBInterface::addActionsState(vector<supervisor_msgs::ActionMS> actions, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/add_facts_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> to_add;
	vector<toaster_msgs::Fact> to_remove;

	for(vector<supervisor_msgs::ActionMS>::iterator it = actions.begin(); it != actions.end(); it++){
		//we remove previous state
		ostringstream to_string;
        	to_string << it->id;
        	string action_id = to_string.str();
		toaster_msgs::Fact fact_rm;
		fact_rm.subjectId = action_id;
		fact_rm.property = "actionState";
		to_remove.push_back(fact_rm);
		//then we add the new state
		toaster_msgs::Fact fact_add;
		fact_add.subjectId = action_id;
		fact_add.property = "actionState";
		fact_add.targetId = state;
		to_add.push_back(fact_add);
	}
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	srv_rm.request.facts = to_remove;
	if (!client_rm.call(srv_rm)){
	 ROS_ERROR("[mental_state] Failed to call service database/remove_fact_to_agent");
	}
	toaster_msgs::AddFactsToAgent srv;
	srv.request.agentId = agent;
	srv.request.facts = to_add;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  	}
}


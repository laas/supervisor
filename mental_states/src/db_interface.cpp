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
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::RemoveFactToAgent>("database/remove_fact_to_agent");
  	ros::ServiceClient client_SQL = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	//retrieve previous state of the goal
	toaster_msgs::ExecuteSQL srvSQL;
	srvSQL.request.order = "SELECT target_id FROM fact_table_";
	srvSQL.request.order = srvSQL.request.order + agent + " WHERE subject_id = '" + goal.name +"' AND predicate = 'goalState'";
  	if (client_SQL.call(srvSQL)){
		if(srvSQL.response.results.size() > 0){
			//we remove previous state
			toaster_msgs::RemoveFactToAgent srv_rm;
			srv_rm.request.agentId = agent;
			toaster_msgs::Fact fact;
			fact.subjectId = goal.name;
			fact.property = "goalState";
			fact.targetId = srvSQL.response.results[0];
			srv_rm.request.fact = fact;
	  		if (!client_rm.call(srv_rm)){
	   		 ROS_ERROR("Failed to call service database/remove_fact_to_agent");
	  		}
		}
	}else{
   	 	ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
  	}
	
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
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::RemoveFactToAgent>("database/remove_fact_to_agent");
  	ros::ServiceClient client_SQL = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	//retrieve previous state of the plan
	toaster_msgs::ExecuteSQL srvSQL;
	ostringstream to_string;
        to_string << plan.id;
        string plan_id = to_string.str();
	srvSQL.request.order = "SELECT target_id FROM fact_table_";
	srvSQL.request.order = srvSQL.request.order + agent + " WHERE subject_id = " + plan_id +" AND predicate = 'planState'";
  	if (client_SQL.call(srvSQL)){
		if(srvSQL.response.results.size() > 0){
			//we remove previous state
			toaster_msgs::RemoveFactToAgent srv_rm;
			srv_rm.request.agentId = agent;
			toaster_msgs::Fact fact;
			fact.subjectId = plan_id;
			fact.property = "planState";
			fact.targetId = srvSQL.response.results[0];
			srv_rm.request.fact = fact;
	  		if (!client_rm.call(srv_rm)){
	   		 ROS_ERROR("[mental_state] Failed to call service database/remove_fact_to_agent");
	  		}
		}
	}else{
   	 	ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
  	}
	
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
void DBInterface::addActionState(supervisor_msgs::ActionMS action, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::RemoveFactToAgent>("database/remove_fact_to_agent");
  	ros::ServiceClient client_SQL = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	//retrieve previous state of the plan
	toaster_msgs::ExecuteSQL srvSQL;
	ostringstream to_string;
        to_string << action.id;
        string action_id = to_string.str();
	srvSQL.request.order = "SELECT target_id FROM fact_table_";
	srvSQL.request.order = srvSQL.request.order + agent + " WHERE subject_id = " + action_id +" AND predicate = 'actionState'";
  	if (client_SQL.call(srvSQL)){
		if(srvSQL.response.results.size() > 0){
			//we remove previous state
			toaster_msgs::RemoveFactToAgent srv_rm;
			srv_rm.request.agentId = agent;
			toaster_msgs::Fact fact;
			fact.subjectId = action_id;
			fact.property = "actionState";
			fact.targetId = srvSQL.response.results[0];
			srv_rm.request.fact = fact;
	  		if (!client_rm.call(srv_rm)){
	   		 ROS_ERROR("[mental_state] Failed to call service database/remove_fact_to_agent");
	  		}
		}
	}else{
   	 	ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
  	}
	
	toaster_msgs::AddFactToAgent srv;
	srv.request.agentId = agent;
	toaster_msgs::Fact fact;
	fact.subjectId = action_id;
	fact.property = "actionState";
	fact.targetId = state;
	srv.request.fact = fact;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_fact_to_agent");
  	}
}


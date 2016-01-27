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
	@state: the state we want to put
*/
void DBInterface::addGoalState(supervisor_msgs::GoalMS goal, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> toRemove;

	//we remove previous state
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	toaster_msgs::Fact factRm;
	factRm.subjectId = goal.name;
	factRm.property = "goalState";
	factRm.targetId = "NULL";
	toRemove.push_back(factRm);
	srv_rm.request.facts = toRemove;
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
	@plan: the plan
	@agent: the agent name 
	@state: the state we want to put
*/
void DBInterface::addPlanState(supervisor_msgs::PlanMS plan, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> toRemove;

	//retrieve previous state of the plan
	toaster_msgs::ExecuteSQL srvSQL;
	ostringstream toString;
   toString << plan.id;
   string planId = toString.str();
	//we remove previous state
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	toaster_msgs::Fact factRm;
	factRm.subjectId = planId;
	factRm.property = "planState";
	factRm.targetId = "NULL";
	toRemove.push_back(factRm);
	srv_rm.request.facts = toRemove;
	if (!client_rm.call(srv_rm)){
	 ROS_ERROR("[mental_state] Failed to call service database/remove_fact_to_agent");
	}
	//then we add the new state
	toaster_msgs::AddFactToAgent srv;
	srv.request.agentId = agent;
	toaster_msgs::Fact fact;
	fact.subjectId = planId;
	fact.property = "planState";
	fact.targetId = state;
	srv.request.fact = fact;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_fact_to_agent");
  	}
}

/*
Function which add the state of actions in an agent knowledge
	@actions: the list of actions
	@agent: the agent name 
	@state: the state we want to put
*/
void DBInterface::addActionsState(vector<supervisor_msgs::ActionMS> actions, string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/add_facts_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> toAdd;
	vector<toaster_msgs::Fact> toRemove;

	for(vector<supervisor_msgs::ActionMS>::iterator it = actions.begin(); it != actions.end(); it++){
		//we remove previous state
		ostringstream toString;
      toString << it->id;
      string actionId = toString.str();
		toaster_msgs::Fact factRm;
		factRm.subjectId = actionId;
		factRm.property = "actionState";
		factRm.targetId = "NULL";
		toRemove.push_back(factRm);
		//then we add the new state
		toaster_msgs::Fact factAdd;
		factAdd.subjectId = actionId;
		factAdd.property = "actionState";
		factAdd.targetId = state;
		toAdd.push_back(factAdd);
	}
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	srv_rm.request.facts = toRemove;
	if (!client_rm.call(srv_rm)){
	 ROS_ERROR("[mental_state] Failed to call service database/remove_facts_to_agent");
	}
	toaster_msgs::AddFactsToAgent srv;
	srv.request.agentId = agent;
	srv.request.facts = toAdd;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  	}
}

/*
Function which return all the actions id with an action state in the knowledge of an agent
	@agent: the agent name 
	@state: the state we are looking for
*/
vector<int> DBInterface::getActionsIdFromState(string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");
	vector<int> ids;

	//we ask to the database the ids of the action of the given state
	string SQLOrder = "SELECT subject_id FROM fact_table_" + agent + " WHERE predicate = 'actionState' AND target_id = '" + state + "'";
	toaster_msgs::ExecuteSQL srv;
	srv.request.order = SQLOrder;
	if (client.call(srv)){
	 for(vector<string>::iterator it = srv.response.results.begin(); it != srv.response.results.end(); it++){
		istringstream buffer(*it);
      int id;
      buffer >> id;
		ids.push_back(id);
	 }
	}else{
	 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
	}

	return ids;
}

/*
Function which return true if all the facts given are in the agent knowledge
	@agent: the agent name 
	@facts: the facts we are looking for
*/
bool DBInterface::factsAreIn(string agent, vector<toaster_msgs::Fact> facts){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AreInTable>("database/are_in_table");
  	ros::ServiceClient clientSQL = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	vector<toaster_msgs::Fact> toCheck, subjectNULL, targetNULL;
	//we first get all facts with NULL in subject or target
	for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
		if(it->subjectId == "NULL"){
			subjectNULL.push_back(*it);
		}else if(it->targetId == "NULL"){
			targetNULL.push_back(*it);
		}else{
			toCheck.push_back(*it);
		}
	}

	//for the facts of subjectId NULL, we check if there is no fact in the agent knowledge with the same property and targetId
	for(vector<toaster_msgs::Fact>::iterator it = subjectNULL.begin(); it != subjectNULL.end(); it++){
		toaster_msgs::ExecuteSQL srvSQL;
		string SQLOrder = "SELECT subject_id FROM fact_table_" + agent + " WHERE predicate = '" + it->property + "' AND target_id = '" + it->targetId + "'";
		srvSQL.request.order = SQLOrder;
		if (clientSQL.call(srvSQL)){
	 		if(srvSQL.response.results.size() != 0){
				return false;
			}
		}else{
			 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
		}
	}

	//for the facts of targetId NULL, we check if there is no fact in the agent knowledge with the same property and subjectId
	for(vector<toaster_msgs::Fact>::iterator it = subjectNULL.begin(); it != subjectNULL.end(); it++){
		toaster_msgs::ExecuteSQL srvSQL;
		string SQLOrder = "SELECT target_id FROM fact_table_" + agent + " WHERE predicate = '" + it->property + "' AND subject_id = '" + it->targetId + "'";
		srvSQL.request.order = SQLOrder;
		if (clientSQL.call(srvSQL)){
	 		if(srvSQL.response.results.size() != 0){
				return false;
			}
		}else{
			 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
		}
	}
	
	toaster_msgs::AreInTable srv;
	srv.request.facts = toCheck;
	srv.request.agent = agent;
	if (client.call(srv)){
	 return srv.response.result;
	}else{
	 ROS_ERROR("[mental_state] Failed to call service database/are_in_table");
	}
	
	return false;
}

/*
Function which return true if all the facts given are in the agent knowledge
	@agent: the agent name 
	@facts: the facts we are looking for
*/
int DBInterface::getAgentIdPlan(string agent){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	//we ask to the database the ids of the action of the given state
	string SQLOrder = "SELECT subject_id FROM fact_table_" + agent + " WHERE predicate = 'planState' AND target_id = 'PROGRESS'";
	toaster_msgs::ExecuteSQL srv;
	srv.request.order = SQLOrder;
	if (client.call(srv)){
	 if(srv.response.results.size() != 0){
		istringstream buffer(srv.response.results[0]);
      int id;
      buffer >> id;
		return id;
	 }
	}else{
	 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
	}

	return -1;
}

/*
Function which remove all knowledge of an agent about actions with a specific state
	@agent: the agent name 
	@state: the state of the actions we want to remove
*/
void DBInterface::removeActionsState(string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> toRemove;

	toaster_msgs::Fact factRm;
	factRm.subjectId = "NULL";
	factRm.property = "actionState";
	factRm.targetId = state;
	toRemove.push_back(factRm);
	toaster_msgs::AddFactsToAgent srv_rm;
	srv_rm.request.agentId = agent;
	srv_rm.request.facts = toRemove;
	if (!client_rm.call(srv_rm)){
	 ROS_ERROR("[mental_state] Failed to call service database/remove_facts_to_agent");
	}
}

/*
Function which return the state of an action in the knowledge of an agent
	@agent: the agent name 
	@state: the action we want the state
*/
string DBInterface::getActionState(string agent, supervisor_msgs::ActionMS action){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	//we ask to the database the ids of the action of the given state
	ostringstream toString;
   toString << action.id;
   string actionId = toString.str();
	string SQLOrder = "SELECT target_id FROM fact_table_" + agent + " WHERE predicate = 'actionState' AND subject_id = '" + actionId + "'";
	toaster_msgs::ExecuteSQL srv;
	srv.request.order = SQLOrder;
	if (client.call(srv)){
	 if(srv.response.results.size() != 0){
		return srv.response.results[0];
	 }else{
		return "UNKNOWN";
	  }
	}else{
	 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
	}
		return "UNKNOWN";
}

/*
Function which remove all knowledge of an agent about actions with a specific state
	@agent: the agent name 
	@state: the state of the actions we want to remove
*/
vector<string> DBInterface::getAgentGoals(string agent, string state){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");
	vector<string> goals;
	

	//we ask to the database the ids of the action of the given state
	string SQLOrder = "SELECT subject_id FROM fact_table_" + agent + " WHERE predicate = 'goalState' AND target_id = '" + state + "'";
	toaster_msgs::ExecuteSQL srv;
	srv.request.order = SQLOrder;
	if (client.call(srv)){
	 return srv.response.results;
	}else{
	 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
	}

	return goals;

}

/*
Function which return all the agents who can see an agent
	@agent: the agent name 
*/
vector<string> DBInterface::getAgentsWhoSee(string agent){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");
	vector<string> agents;
	string robotName;
  	node.getParam("/robot/name", robotName);
	
	//we ask to the database the ids of the action of the given state
	string SQLOrder = "SELECT target_id FROM fact_table_" + robotName + " WHERE predicate = 'isVisibleBy' AND subject_id = '" + agent + "'";
	toaster_msgs::ExecuteSQL srv;
	srv.request.order = SQLOrder;
	if (client.call(srv)){
	 return srv.response.results;
	}else{
	 ROS_ERROR("[mental_state] Failed to call service database/execute_SQL");
	}


  	return agents;
}

/*
Function which add facts to the knowledge of an agent
	@facts: the facts to add
	@agent: the agent name 
*/
void DBInterface::addFacts(vector<toaster_msgs::Fact> facts, string agent){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/add_facts_to_agent");

	toaster_msgs::AddFactsToAgent srv;
	srv.request.agentId = agent;
	srv.request.facts = facts;
  	if (!client.call(srv)){
   	 ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
  	}
}

/*
Function which add effects of an actions to the knowledge of an agent
	@facts: the effects to add
	@agent: the agent name 
*/
void DBInterface::addEffects(vector<toaster_msgs::Fact> facts, string agent){

	ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/add_facts_to_agent");
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	vector<toaster_msgs::Fact> toAdd;
	vector<toaster_msgs::Fact> toRemove;

	//if there is NULL in the description of the effect, we remove all facts of this type on the knowledge of the agent
	for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
		if(it->subjectId == "NULL" || it->targetId == "NULL"){
			toRemove.push_back(*it);
		}else{
			toAdd.push_back(*it);
		}
	}

	if(toRemove.size() != 0){
		toaster_msgs::AddFactsToAgent srv_rm;
		srv_rm.request.agentId = agent;
		srv_rm.request.facts = toRemove;
		if (!client_rm.call(srv_rm)){
		 ROS_ERROR("[mental_state] Failed to call service database/remove_facts_to_agent");
		}
	}

	if(toAdd.size() != 0){
		toaster_msgs::AddFactsToAgent srv;
		srv.request.agentId = agent;
		srv.request.facts = toAdd;
	  	if (!client.call(srv)){
	   	 ROS_ERROR("[mental_state] Failed to call service database/add_facts_to_agent");
	  	}
	}
}

/*
Function which return all the knowledge of an agent
	@agent: the agent name 
*/
vector<toaster_msgs::Fact> DBInterface::getFactsAgent(string agent){

   ros::NodeHandle node;
  	ros::ServiceClient client = node.serviceClient<toaster_msgs::GetFacts>("database/get_current_facts_from_agent");
	toaster_msgs::GetFacts srv;
	
	
	srv.request.agentId = agent;
	if (client.call(srv)){
	   return srv.response.resFactList.factList;
	}else{
	   ROS_ERROR("[mental_state] Failed to call service database/get_current_facts_from_agent");
	}
}


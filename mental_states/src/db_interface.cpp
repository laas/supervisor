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
Function which add a fact to the list to add
    @fact: the fact to add
    @agent: the agent which goes with the fact
*/
void DBInterface::addFactToAdd(toaster_msgs::Fact fact, string agent){

    //We look if the agent already exist in the list to add
    bool find = false;
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = toAdd_.begin(); it != toAdd_.end(); it++){
        if(it->first == agent){//if it exist we add the fact to the list
            it->second.push_back(fact);
            find = true;
        }
    }
    if(!find){//if it does not exist we add it
        pair<string, vector<toaster_msgs::Fact> > toAddAgent;
        toAddAgent.first = agent;
        toAddAgent.second.push_back(fact);
        toAdd_.push_back(toAddAgent);
    }

    //update knowledge
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            it->second.push_back(fact);
        }
    }

}

/*
Function which add a fact to the list to remove
    @fact: the fact to add
    @agent: the agent which goes with the fact
*/
void DBInterface::addFactToRemove(toaster_msgs::Fact fact, string agent){

    //We look if the agent already exist in the list to add
    bool find = false;
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = toRemove_.begin(); it != toRemove_.end(); it++){
        if(it->first == agent){//if it exist we add the fact to the list
            it->second.push_back(fact);
            find = true;
        }
    }
    if(!find){//if it does not exist we add it
        pair<string, vector<toaster_msgs::Fact> > toRemoveAgent;
        toRemoveAgent.first = agent;
        toRemoveAgent.second.push_back(fact);
        toRemove_.push_back(toRemoveAgent);
    }

    //update knowledge
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
             vector<toaster_msgs::Fact> tmp;
            for(vector<toaster_msgs::Fact>::iterator itf = it->second.begin(); itf != it->second.end(); itf++){
                if((fact.subjectId == itf->subjectId && fact.property == itf->property && fact.targetId == itf->targetId)
                        || (fact.subjectId == "NULL" && fact.property == itf->property && fact.targetId == itf->targetId)
                        || (fact.subjectId == itf->subjectId && fact.property == itf->property && fact.targetId == "NULL")
                        || (fact.subjectId == "NULL" && fact.property == itf->property && fact.targetId == "NULL")){

                }else{
                    tmp.push_back(*itf);
                }
            }
            it->second = tmp;
        }
    }

}


/*
Function which init the knowledge of agent with the database
*/
void DBInterface::initKnowledge(vector<string> agents){

    for(vector<string>::iterator it = agents.begin(); it != agents.end(); it++){
        pair<string, vector<toaster_msgs::Fact> > agentKnowledge;
        agentKnowledge.first = *it;
        agentKnowledge.second = getFactsAgent(*it);
        knowledge_.push_back(agentKnowledge);
    }
}

/*
Function which send update to the database and update the mental states.
*/
void DBInterface::updateKnowledge(){

    //we send to the database the last update
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = toRemove_.begin(); it != toRemove_.end(); it++){
        removeFacts(it->second, it->first);
    }
    toRemove_.clear();
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = toAdd_.begin(); it != toAdd_.end(); it++){
        addFacts(it->second, it->first);
    }
    toAdd_.clear();

    //we get the last update from the database
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        it->second = getFactsAgent(it->first);
    }

}


/*
Function which add the state of a goal in an agent knowledge
	@goal: the goal
	@agent: the agent name 
	@state: the state we want to put
*/
void DBInterface::addGoalState(supervisor_msgs::GoalMS goal, string agent, string state){

    //we create the fact to remove and add it to the list
    toaster_msgs::Fact factRm;
    factRm.subjectId = goal.name;
    factRm.property = "goalState";
    factRm.propertyType = "state";
    factRm.targetId = "NULL";
    factRm.factObservability = 0.0;
    addFactToRemove(factRm, agent);

    //Then we create the fact to add and add it to the list
    toaster_msgs::Fact factAdd;
    factAdd.subjectId = goal.name;
    factAdd.property = "goalState";
    factAdd.propertyType = "state";
    factAdd.targetId = state;
    factAdd.factObservability = 0.0;
    addFactToAdd(factAdd, agent);



}

/*
Function which add the state of a plan in an agent knowledge
	@plan: the plan
	@agent: the agent name 
	@state: the state we want to put
*/
void DBInterface::addPlanState(supervisor_msgs::PlanMS plan, string agent, string state){

    //we get the plan id
    ostringstream toString;
    toString << plan.id;
    string planId = toString.str();
    //we remove previous state
    toaster_msgs::Fact factRm;
    factRm.subjectId = planId;
    factRm.property = "planState";
    factRm.propertyType = "state";
    factRm.targetId = "NULL";
    factRm.factObservability = 0.0;
    addFactToRemove(factRm, agent);

    //then we add the new state
    toaster_msgs::Fact fact;
    fact.subjectId = planId;
    fact.property = "planState";
    fact.propertyType = "state";
    fact.targetId = state;
    fact.factObservability = 0.0;
    addFactToAdd(fact, agent);

}

/*
Function which add the state of actions in an agent knowledge
	@actions: the list of actions
	@agent: the agent name 
	@state: the state we want to put
*/
void DBInterface::addActionsState(vector<supervisor_msgs::ActionMS> actions, string agent, string state){

    for(vector<supervisor_msgs::ActionMS>::iterator it = actions.begin(); it != actions.end(); it++){
        //we remove previous state
        ostringstream toString;
        toString << it->id;
        string actionId = toString.str();
        toaster_msgs::Fact factRm;
        factRm.subjectId = actionId;
        factRm.property = "actionState";
        factRm.propertyType = "state";
        factRm.targetId = "NULL";
        factRm.factObservability = 0.0;
        addFactToRemove(factRm, agent);
        //then we add the new state
        toaster_msgs::Fact factAdd;
        factAdd.subjectId = actionId;
        factAdd.property = "actionState";
        factAdd.propertyType = "state";
        factAdd.targetId = state;
        factAdd.factObservability = 0.0;
        addFactToAdd(factAdd, agent);
    }

}

/*
Function which return all the actions id with an action state in the knowledge of an agent
	@agent: the agent name 
	@state: the state we are looking for
*/
vector<int> DBInterface::getActionsIdFromState(string agent, string state){

    vector<int> toReturn;
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "actionState" && itk->targetId == state){
                    istringstream buffer(itk->subjectId);
                    int id;
                    buffer >> id;
                    toReturn.push_back(id);
                }
            }
        }
    }

    return toReturn;
}

/*
Function which return true if all the facts given are in the agent knowledge
	@agent: the agent name 
	@facts: the facts we are looking for
*/
bool DBInterface::factsAreIn(string agent, vector<toaster_msgs::Fact> facts){

    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itf = facts.begin(); itf != facts.end(); itf++){
                if(itf->subjectId == "NULL"){
                    for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                        if(itk->property == itf->property && itk->targetId == itf->targetId){
                            return false;
                        }
                    }
                }else if(itf->targetId == "NULL"){
                    for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                        if(itk->property == itf->property && itk->subjectId == itf->subjectId){
                            return false;
                        }
                    }
                }else{
                    bool find = false;
                    for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                        if(itk->property == itf->property && itk->subjectId == itf->subjectId && itk->targetId == itf->targetId){
                            find = true;
                            break;
                        }
                    }
                    if(!find){
                        return false;
                    }
                }
            }
            return true;
        }
    }
    return false; //there is no knowledge concerning this agent
}

/*
Function which return the id of the plan in PROGRESS fo the agent
    @agent: the agent name
*/
int DBInterface::getAgentIdPlan(string agent){

    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "planState" && itk->targetId == "PROGRESS"){
                    istringstream buffer(itk->subjectId);
                    int id;
                    buffer >> id;
                    return id;
                }
            }
        }
    }
    return -1;
}

/*
Function which remove all knowledge of an agent about actions with a specific state
	@agent: the agent name 
	@state: the state of the actions we want to remove
*/
void DBInterface::removeActionsState(string agent, string state){

    toaster_msgs::Fact factRm;
    factRm.subjectId = "NULL";
    factRm.property = "actionState";
    factRm.propertyType = "state";
    factRm.targetId = state;
    factRm.factObservability = 0.0;
    addFactToRemove(factRm, agent);

}

/*
Function which return the state of an action in the knowledge of an agent
	@agent: the agent name 
    @action: the action we want the state
*/
string DBInterface::getActionState(string agent, supervisor_msgs::ActionMS action){

    stringstream toString;
    toString << action.id;
    string actionId = toString.str();
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "actionState" && itk->subjectId == actionId){
                    return itk->targetId;
                }
            }
        }
    }
    return "UNKNOWN";
}

/*
Function which return all the goals with a specific state
	@agent: the agent name 
    @state: the state of the goals wanted
*/
vector<string> DBInterface::getAgentGoals(string agent, string state){

    vector<string> goals;
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "goalState" && itk->targetId == state){
                    goals.push_back(itk->subjectId);
                }
            }
        }
    }
    return goals;
}

/*
Function which return all the agents who can see an agent
	@agent: the agent name 
*/
vector<string> DBInterface::getAgentsWhoSee(string agent){
    ros::NodeHandle node;
    vector<string> agents;
    string robotName;
    node.getParam("/robot/name", robotName);
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == robotName){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "isVisibleBy" && itk->subjectId == agent){
                    agents.push_back(itk->targetId);
                }
            }
        }
    }
    return agents;
}

/*
Function which return true if an agent can see another agent
    @agentTested: the agent who should see the other
    @agentToSee: the agent who needs to be seen
*/
bool DBInterface::isAgentSeeing(string agentTested, string agentToSee){
    ros::NodeHandle node;
    string robotName;
    node.getParam("/robot/name", robotName);
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == robotName){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "isVisibleBy" && itk->subjectId == agentToSee && itk->targetId == agentTested){
                    return true;
                }
            }
        }
    }
    return false;

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
Function which remove facts to the knowledge of an agent
    @facts: the facts to remove
    @agent: the agent name
*/
void DBInterface::removeFacts(vector<toaster_msgs::Fact> facts, string agent){

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");

    toaster_msgs::AddFactsToAgent srv;
    srv.request.agentId = agent;
    srv.request.facts = facts;
    if (!client.call(srv)){
     ROS_ERROR("[mental_state] Failed to call service database/remove_facts_to_agent");
    }
}

/*
Function which add effects of an actions to the knowledge of an agent
	@facts: the effects to add
	@agent: the agent name 
*/
void DBInterface::addEffects(vector<toaster_msgs::Fact> facts, string agent){

    ros::NodeHandle node;
    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        string obsTopic = "observableFacts/";
        obsTopic = obsTopic + it->property;
        bool isObservable;
        if(node.hasParam(obsTopic)){
            node.getParam(obsTopic, isObservable);
        }else{
            isObservable = false;
        }
        if(isObservable){
            it->factObservability = 1.0;
        }else{
            it->factObservability = 0.0;
        }
        if(it->subjectId == "NULL" || it->targetId == "NULL"){
            //if there is NULL in the description of the effect, we remove all facts of this type on the knowledge of the agent
            addFactToRemove(*it, agent);
        }else{
            addFactToAdd(*it, agent);
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

/*
Function which remove all knowledge from the database
	@agent: the agent name 
*/
void DBInterface::cleanDB(){

   ros::NodeHandle node;
  	ros::ServiceClient client_rm = node.serviceClient<toaster_msgs::AddFactsToAgent>("database/remove_facts_to_agent");
	toaster_msgs::AddFactsToAgent srv_rm;
	vector<toaster_msgs::Fact> toRemove;
	
	vector<string> allAgents = getAgents();
	
	toaster_msgs::Fact factAct;
	factAct.subjectId = "NULL";
	factAct.property = "actionState";
	factAct.propertyType = "state";
	factAct.targetId = "NULL";
    factAct.factObservability = 0.0;
	toRemove.push_back(factAct);
	toaster_msgs::Fact factPlan;
	factPlan.subjectId = "NULL";
	factPlan.property = "planState";
	factPlan.propertyType = "state";
	factPlan.targetId = "NULL";
    factPlan.factObservability = 0.0;
	toRemove.push_back(factPlan);
	toaster_msgs::Fact factGoal;
	factGoal.subjectId = "NULL";
	factGoal.property = "goalState";
	factGoal.propertyType = "state";
	factGoal.targetId = "NULL";
    factGoal.factObservability = 0.0;
	toRemove.push_back(factGoal);
	srv_rm.request.facts = toRemove;
	   
	for(vector<string>::iterator it = allAgents.begin(); it != allAgents.end(); it++){
	   srv_rm.request.agentId = *it;
	   if (!client_rm.call(srv_rm)){
		    ROS_ERROR("[mental_state] Failed to call service database/remove_facts_to_agent");
	   }
	}
}

/*
Function which return the state of a goal
	@agent: the agent for who we want the goal state
	@goal: the goal 
*/
string DBInterface::getGoalState(string agent, supervisor_msgs::GoalMS goal){

    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "goalState" && itk->subjectId == goal.name){
                    return itk->targetId;
                }
            }
        }
    }
    return "NULL";
}

/*
Function which return the state of a plan
    @agent: the agent for who we want the plan state
    @plan: the plan
*/
string DBInterface::getPlanState(string agent, supervisor_msgs::PlanMS plan){

    stringstream toString;
    toString << plan.id;
    string planId = toString.str();
    for(vector<pair<string, vector<toaster_msgs::Fact> > >::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->first == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->second.begin(); itk != it->second.end(); itk++){
                if(itk->property == "planState" && itk->subjectId == planId){
                    return itk->targetId;
                }
            }
        }
    }
    return "NULL";
}

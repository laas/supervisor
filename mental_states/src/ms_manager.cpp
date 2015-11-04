/**
author Sandra Devin

Class which allows to maintain mental states of the agents
**/

#include <mental_states/ms_manager.h>

MSManager::MSManager(){
	actionId = 0;
	planId = 0;
}

/*
Function which initialize the list of goals from the goal in param
*/
void MSManager::initGoals(){
	ros::NodeHandle node;
	//we retrieve the possible goals from param of the .yaml file
	vector<string> names;
	node.getParam("/goals/names", names);
	for(vector<string>::iterator it = names.begin(); it != names.end(); it++){
		//for each goal we retreive its actors and objective
		string actors_topic = "goals/";
		actors_topic = actors_topic + *it + "_actors";
		vector<string> actors;
		node.getParam(actors_topic, actors);
		string objective_topic = "goals/";
		objective_topic = objective_topic + *it + "_objective";
		vector<string> objective;
		node.getParam(objective_topic, objective);
		//we transform the objectiv into facts
		vector<toaster_msgs::Fact> obj;
		for(vector<string>::iterator ob = objective.begin(); ob != objective.end(); ob++){
			int beg = ob->find(',');
    			int end = ob->find(',', beg+1);
			toaster_msgs::Fact fact;
    			fact.subjectId = ob->substr(0, beg);
    			fact.property = ob->substr(beg+1, end - beg - 1);
    			fact.targetId = ob->substr(end+1, ob->size() - end - 1);
			obj.push_back(fact);
		}
		//we then fill the goal list
		supervisor_msgs::GoalMS goal;
		goal.name = *it;
		goal.actors = actors;
		goalList.push_back(goal);
	}
	
}

/*
Function which initialize the list of high level actions from param
*/
void MSManager::initHighLevelActions(){
	ros::NodeHandle node;
	//we retrieve the high level actions from param of the .yaml file
	vector<string> names;
	node.getParam("/highLevelActions/names", names);
	for(vector<string>::iterator it = names.begin(); it != names.end(); it++){
		//for each high level action we retreive its composition
		string param_topic = "highLevelActions/";
		param_topic = param_topic + *it + "_param";
		vector<string> params;
		node.getParam(param_topic, params);
		string actors_topic = "highLevelActions/";
		actors_topic = actors_topic + *it + "_actors";
		vector<string> actors;
		node.getParam(actors_topic, actors);
		string prec_topic = "highLevelActions/";
		prec_topic = prec_topic + *it + "_prec";
		vector<string> prec;
		node.getParam(prec_topic, prec);
		//we transform the preconditions into facts
		vector<toaster_msgs::Fact> prec_fact;
		for(vector<string>::iterator p = prec.begin(); p != prec.end(); p++){
			int beg = p->find(',');
    			int end = p->find(',', beg+1);
			toaster_msgs::Fact fact;
    			fact.subjectId = p->substr(0, beg);
    			fact.property = p->substr(beg+1, end - beg - 1);
    			fact.targetId = p->substr(end+1, p->size() - end - 1);
			prec_fact.push_back(fact);
		}
		string effects_topic = "highLevelActions/";
		effects_topic = effects_topic + *it + "_effects";
		vector<string> effects;
		node.getParam(effects_topic, effects);
		//we transform the effects into facts
		vector<toaster_msgs::Fact> effects_fact;
		for(vector<string>::iterator e = effects.begin(); e != effects.end(); e++){
			int beg = e->find(',');
    			int end = e->find(',', beg+1);
			toaster_msgs::Fact fact;
    			fact.subjectId = e->substr(0, beg);
    			fact.property = e->substr(beg+1, end - beg - 1);
    			fact.targetId = e->substr(end+1, e->size() - end - 1);
			effects_fact.push_back(fact);
		}
		//we then fill the high level action list
		supervisor_msgs::ActionMS highLevelAction;
		highLevelAction.name = *it;
		highLevelAction.parameters = params;
		highLevelAction.actors = actors;
		highLevelAction.prec = prec_fact;
		highLevelAction.effects = effects_fact;
		highLevelActions.push_back(highLevelAction);
	}
	
}

/*
Function which updates the mental state of an agent
	@agent: the name of the agent
*/
void MSManager::update(string agent){
	
	//We check if the agent can deduce an action from its effects.
 	checkEffects(agent);
	//We check if the precondition and links of the planned action.
        computePreconditions(agent);
	//We check if the agent still think that the current plan is still feasible (and not achieved too).
        planFeasibility(agent);
	//We check if the agent still think that the current goal is stillfeasible (and not achieved too).
        checkGoals(agent);

}

/*
Function which checks if an agent can deduce an action from its effects.
	@agent: the name of the agent
*/
void MSManager::checkEffects(string agent){

	//We get all the actions either READY, in PROGRESS, NEEDED or ASKED in the agent knowledge.

	//For each of these actions we look if its effects are in the knowledge of the agent

	//If they are, the action becomes DONE in the agent knowledge

}

/*
Function which checks the precondition and links of the planned action.
	@agent: the name of the agent
*/
void MSManager::computePreconditions(string agent){


}

/*
Function which checks if an agent still think that the current plan is still feasible (and not achieved too).
	@agent: the name of the agent
*/
void MSManager::planFeasibility(string agent){


}


/*
Function which checks if an agent still think that the current goal is still feasible (and not achieved too).
	@agent: the name of the agent
*/
void MSManager::checkGoals(string agent){

}

/*
Function which create an ActionMS from an other action based on preconditions and effects of High level actions
	@action: action without preconditions and effects (Action format)
*/
supervisor_msgs::ActionMS MSManager::getHighLevelActionByName(string name){

	supervisor_msgs::ActionMS action;

	for(vector<supervisor_msgs::ActionMS>::iterator it = highLevelActions.begin(); it != highLevelActions.end(); it++){
		if(it->name == name){
			return *it;
		}
	}

	ROS_ERROR("[mental_state] Unknown action name");
	return action;

}

int MSManager::getAndIncreasePlanId(){
	planId++;
	return (planId - 1);

}

/*
Function which create an ActionMS from an other action based on preconditions and effects of High level actions
	@action: action without preconditions and effects (Action format)
*/
supervisor_msgs::ActionMS MSManager::createActionFromHighLevel(supervisor_msgs::Action action){

	supervisor_msgs::ActionMS newAction;
	//We map the name of parameters and actors with the name from the high level action
	supervisor_msgs::ActionMS highLevelAction = getHighLevelActionByName(action.name);
	map<string, string> highLevelNames;
	highLevelNames["NULL"] = "NULL";
	if(newAction.parameters.size() != highLevelAction.parameters.size()){
		vector<string>::iterator it2 = newAction.parameters.begin();
		for(vector<string>::iterator it = highLevelAction.parameters.begin(); it != highLevelAction.parameters.end(); it++){
			highLevelNames[*it] = *it2;
			it2++;
		}
	}else{
		ROS_ERROR("[mental_state] Incorrect number of parameters");
		return newAction;
	}
	if(newAction.actors.size() != highLevelAction.actors.size()){
		vector<string>::iterator it2 = newAction.actors.begin();
		for(vector<string>::iterator it = highLevelAction.actors.begin(); it != highLevelAction.actors.end(); it++){
			highLevelNames[*it] = *it2;
			it2++;
		}

	}else{
		ROS_ERROR("[mental_state] Incorrect number of actors");
		return newAction;
	}

	//We fill the action
	newAction.name = action.name;
	newAction.id = actionId;
	actionId ++;
	newAction.parameters = action.parameters;
	newAction.actors = action.actors;
	for(vector<toaster_msgs::Fact>::iterator it = highLevelAction.prec.begin(); it != highLevelAction.prec.end(); it++){
		toaster_msgs::Fact fact;
		fact.subjectId = highLevelNames[it->subjectId];
		fact.targetId = highLevelNames[it->targetId];
		fact.property = highLevelNames[it->property];
		newAction.prec.push_back(fact);
	}
	for(vector<toaster_msgs::Fact>::iterator it = highLevelAction.effects.begin(); it != highLevelAction.effects.end(); it++){
		toaster_msgs::Fact fact;
		fact.subjectId = highLevelNames[it->subjectId];
		fact.targetId = highLevelNames[it->targetId];
		fact.property = highLevelNames[it->property];
		newAction.effects.push_back(fact);
	}
	

	return newAction;

}

/*
Function which return a goal giving its name
	@name: name of the goal
*/
supervisor_msgs::GoalMS* MSManager::getGoalByName(string name){

	supervisor_msgs::GoalMS* goal = NULL;

	boost::unique_lock<boost::mutex> lock(goalList_mutex);
	
	for(vector<supervisor_msgs::GoalMS>::iterator it = goalList.begin(); it != goalList.end(); it++){
		if(it->name == name)
			return &(*it);
	}

	return goal;
}


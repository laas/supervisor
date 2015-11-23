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
		goal.objective = obj;
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
    			fact.property = p->substr(beg+2, end - beg - 2);
    			fact.targetId = p->substr(end+2, p->size() - end - 2);
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
    			fact.property = e->substr(beg+2, end - beg - 2);
    			fact.targetId = e->substr(end+2, e->size() - end - 2);
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
	//We check if the agent still think that the current goal is still feasible (and not achieved too).
        checkGoals(agent);

}

/*
Function which checks if an agent can deduce an action from its effects.
	@agent: the name of the agent
*/
void MSManager::checkEffects(string agent){

	DBInterface db;

	//We get all the actions either READY, in PROGRESS, NEEDED or ASKED in the agent knowledge.
	vector<int> readyIds = db.getActionsIdFromState(agent, "READY");
	vector<supervisor_msgs::ActionMS> readyActions = getActionsFromIds(readyIds);
	vector<int> progressIds = db.getActionsIdFromState(agent, "PROGRESS");
	vector<supervisor_msgs::ActionMS> progressActions = getActionsFromIds(progressIds);
	vector<int> neededIds = db.getActionsIdFromState(agent, "NEEDED");
	vector<supervisor_msgs::ActionMS> neededActions = getActionsFromIds(neededIds);
	vector<int> askedIds = db.getActionsIdFromState(agent, "ASKED");
	vector<supervisor_msgs::ActionMS> askedActions = getActionsFromIds(askedIds);

	vector<supervisor_msgs::ActionMS> toCheckActions;
	toCheckActions.insert(toCheckActions.end(), progressActions.begin(), progressActions.end() );
	toCheckActions.insert(toCheckActions.end(), neededActions.begin(), neededActions.end() );
	toCheckActions.insert(toCheckActions.end(), askedActions.begin(), askedActions.end() );
	toCheckActions.insert(toCheckActions.end(), readyActions.begin(), readyActions.end() );

	//For each of these actions we look if its effects are in the knowledge of the agent, if they are, the action becomes DONE in the agent knowledge
	vector<supervisor_msgs::ActionMS> to_done;
	for(vector<supervisor_msgs::ActionMS>::iterator it = toCheckActions.begin(); it != toCheckActions.end(); it++){
		if(db.factsAreIn(agent, it->effects)){
			to_done.push_back(*it);
		}
	}

	db.addActionsState(to_done, agent, "DONE");
}

/*
Function which checks the precondition and links of the planned action.
	@agent: the name of the agent
*/
void MSManager::computePreconditions(string agent){

	DBInterface db;

	pair<bool, supervisor_msgs::PlanMS> agentPlan = getAgentPlan(agent);
	if(agentPlan.first){
		//Check if the READY actions are still READY
		vector<int> readyIds = db.getActionsIdFromState(agent, "READY");
		vector<supervisor_msgs::ActionMS> readyActions = getActionsFromIds(readyIds);
		vector<supervisor_msgs::ActionMS> to_needed;
		vector<supervisor_msgs::ActionMS> to_ready;
		for(vector<supervisor_msgs::ActionMS>::iterator it = readyActions.begin(); it != readyActions.end(); it++){
			if(!db.factsAreIn(agent, it->prec)){ //if the preconditions are not respected anymore, the action goes back to NEEDED
				to_needed.push_back(*it);
			}
		}

		//Check causaul links of PLANNED actions
		vector<int> plannedIds = db.getActionsIdFromState(agent, "PLANNED");
		vector<int> linksOkIds;
		vector<supervisor_msgs::ActionMS> linksOk;
		if(plannedIds.size() != 0){
			for(vector<int>::iterator it = plannedIds.begin(); it != plannedIds.end(); it++){
				vector<toaster_msgs::Fact> to_check;
				//we get all actions id with a link to the action
				for(vector<supervisor_msgs::Link>::iterator itl = agentPlan.second.links.begin(); itl != agentPlan.second.links.end(); itl++){
					if(itl->following == *it){
						toaster_msgs::Fact fact;
						ostringstream to_string;
						to_string << itl->origin;
						fact.subjectId = to_string.str();
						fact.property = "actionState";
						fact.targetId = "DONE";
						to_check.push_back(fact);
					}
				}
				//we check the agent thinks all these actions are DONE
				if(db.factsAreIn(agent, to_check)){
					linksOkIds.push_back(*it);
				}
			}
			linksOk = getActionsFromIds(linksOkIds);
		}

		//Check preconditions of remaining actions + NEEDED actions
		vector<int> neededIds = db.getActionsIdFromState(agent, "NEEDED");
		vector<supervisor_msgs::ActionMS> neededActions = getActionsFromIds(neededIds);
		for(vector<supervisor_msgs::ActionMS>::iterator it = linksOk.begin(); it != linksOk.end(); it++){
			if(db.factsAreIn(agent, it->prec)){ //if the preconditions are respected the action becomes READY, else it goes to NEEDED
				to_ready.push_back(*it);
			}else{
				to_needed.push_back(*it);
			}
		}
		for(vector<supervisor_msgs::ActionMS>::iterator it = neededActions.begin(); it != neededActions.end(); it++){
			if(db.factsAreIn(agent, it->prec)){ //if the preconditions are respected the action becomes READY, else it remains to NEEDED
				to_ready.push_back(*it);
			}
		}

		db.addActionsState(to_needed, agent, "NEEDED");
		db.addActionsState(to_ready, agent, "READY");
	}
}

/*
Function which checks if an agent still think that the current plan is still feasible (and not achieved too).
	@agent: the name of the agent
*/
void MSManager::planFeasibility(string agent){

	DBInterface db;
	
	pair<bool, supervisor_msgs::PlanMS> agentPlan = getAgentPlan(agent);
	if(agentPlan.first){
		//We get all the actions either READY, in PROGRESS, NEEDED, ASKED and PLANNED in the agent knowledge.
		vector<int> readyIds = db.getActionsIdFromState(agent, "READY");
		vector<supervisor_msgs::ActionMS> readyActions = getActionsFromIds(readyIds);
		vector<int> progressIds = db.getActionsIdFromState(agent, "PROGRESS");
		vector<supervisor_msgs::ActionMS> progressActions = getActionsFromIds(progressIds);
		vector<int> neededIds = db.getActionsIdFromState(agent, "NEEDED");
		vector<supervisor_msgs::ActionMS> neededActions = getActionsFromIds(neededIds);
		vector<int> plannedIds = db.getActionsIdFromState(agent, "PLANNED");
		vector<supervisor_msgs::ActionMS> plannedActions = getActionsFromIds(plannedIds);
		vector<int> askedIds = db.getActionsIdFromState(agent, "ASKED");
		vector<supervisor_msgs::ActionMS> askedActions = getActionsFromIds(askedIds);

		//if there is no actions, the plan is done
		vector<supervisor_msgs::ActionMS> allActions;
		allActions.insert(allActions.end(), progressActions.begin(), progressActions.end() );
		allActions.insert(allActions.end(), neededActions.begin(), neededActions.end() );
		allActions.insert(allActions.end(), askedActions.begin(), askedActions.end() );
		allActions.insert(allActions.end(), plannedActions.begin(), plannedActions.end() );
		allActions.insert(allActions.end(), readyActions.begin(), readyActions.end() );
		if(allActions.size() == 0){
			db.addPlanState(agentPlan.second, agent, "DONE");
		}else{ //else, if there is no action in progress or possible the plan is aborted
			vector<supervisor_msgs::ActionMS> possibleActions;
			possibleActions.insert(possibleActions.end(), progressActions.begin(), progressActions.end() );
			possibleActions.insert(possibleActions.end(), askedActions.begin(), askedActions.end() );
			possibleActions.insert(possibleActions.end(), readyActions.begin(), readyActions.end() );
			if(possibleActions.size() == 0){
				abortPlan(agent);
			}
		}
	}

}


/*
Function which checks if an agent still think that the current goal is still feasible (and not achieved too).
	@agent: the name of the agent
*/
void MSManager::checkGoals(string agent){

	DBInterface db;

	//We first get all goals in PROGRESS or PENDING in the knowledge of the agent
	vector<string> pendingGoals = db.getAgentGoals(agent, "PENDING");
	vector<string> progressGoals = db.getAgentGoals(agent, "PROGRESS");

	//Then we check if objectives of the goal are in the agent knowledge
	for(vector<string>::iterator it = pendingGoals.begin(); it != pendingGoals.end(); it++){
		supervisor_msgs::GoalMS* goal = getGoalByName(*it);
		if(db.factsAreIn(agent, goal->objective)){ 
			db.addGoalState(*goal, agent, "DONE");
		}
	}
	for(vector<string>::iterator it = progressGoals.begin(); it != progressGoals.end(); it++){
		supervisor_msgs::GoalMS* goal = getGoalByName(*it);
		if(db.factsAreIn(agent, goal->objective)){ 
			db.addGoalState(*goal, agent, "DONE");
			abortPlan(agent);//for the current goal, we abort the current plan as it is over
		}
	}


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
	if(action.parameters.size() == highLevelAction.parameters.size()){
		vector<string>::iterator it2 = action.parameters.begin();
		for(vector<string>::iterator it = highLevelAction.parameters.begin(); it != highLevelAction.parameters.end(); it++){
			highLevelNames[*it] = *it2;
			it2++;
		}
	}else{
		ROS_ERROR("[mental_state] Incorrect number of parameters: should be %i but is %i!", (int)highLevelAction.parameters.size(), (int)action.parameters.size());
		return newAction;
	}
	if(action.actors.size() == highLevelAction.actors.size()){
		vector<string>::iterator it2 = action.actors.begin();
		for(vector<string>::iterator it = highLevelAction.actors.begin(); it != highLevelAction.actors.end(); it++){
			highLevelNames[*it] = *it2;
			it2++;
		}

	}else{
		ROS_ERROR("[mental_state] Incorrect number of actors: should be %i but is %i!", (int)highLevelAction.actors.size(), (int)action.actors.size());
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
		fact.property = it->property;
		newAction.prec.push_back(fact);
	}
	for(vector<toaster_msgs::Fact>::iterator it = highLevelAction.effects.begin(); it != highLevelAction.effects.end(); it++){
		toaster_msgs::Fact fact;
		fact.subjectId = highLevelNames[it->subjectId];
		fact.targetId = highLevelNames[it->targetId];
		fact.property = it->property;
		newAction.effects.push_back(fact);
	}

	//Add the action to the list of actions
	boost::unique_lock<boost::mutex> lock(actionList_mutex);
	actionList.push_back(newAction);
	

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


/*
Function which return the action of the ids given in paramaters
	@ids: list of id we want the actions
*/
vector<supervisor_msgs::ActionMS> MSManager::getActionsFromIds(vector<int> ids){

	boost::unique_lock<boost::mutex> lock(actionList_mutex);
	
	vector<supervisor_msgs::ActionMS> actions;
	for(vector<int>::iterator it = ids.begin(); it != ids.end(); it++){
		actions.push_back(actionList[*it]);
	}
	
	return actions;
}

/*
Function which add a plan to the plan list
	@plan: the plan to add
*/
void MSManager::addPlanToList(supervisor_msgs::PlanMS plan){

	boost::unique_lock<boost::mutex> lock(planList_mutex);
	planList.push_back(plan);
}

/*
Function which return the plan an agent think in PROGRESS (bool is at true if there is a plan, false otherwize)
	@agent: the agent name
*/
pair<bool, supervisor_msgs::PlanMS> MSManager::getAgentPlan(string agent){
	
	DBInterface db;
	int planId = db.getAgentIdPlan(agent);// we get the id of the plan
	pair<bool, supervisor_msgs::PlanMS> answer;
	boost::unique_lock<boost::mutex> lock(planList_mutex);
	//we return the corresponding plan
	if(planId != -1){
		answer.first = true;
		answer.second = planList[planId];
		return answer;
	}else{
		answer.first = false;
		return answer;
	}
}

/*
Function which aborts the current plan in the knowledge of the agent (and remove PLANNED and NEEDED actions)
	@agent: the name of the agent
*/
void MSManager::abortPlan(string agent){
	
	DBInterface db;
	
	pair<bool, supervisor_msgs::PlanMS> agentPlan = getAgentPlan(agent);
	if(agentPlan.first){
		db.addPlanState(agentPlan.second, agent, "ABORTED");
		db.removeActionsState(agent, "PLANNED");
		db.removeActionsState(agent, "NEEDED");
		db.removeActionsState(agent, "READY");
	}
}

/*
Function which return the action (ActionMS format) corresponding to the action in parameters
	@action: the action in Action format
*/
pair<bool, supervisor_msgs::ActionMS> MSManager::getActionFromAction(supervisor_msgs::Action action){

	boost::unique_lock<boost::mutex> lock(actionList_mutex);
	pair<bool, supervisor_msgs::ActionMS> answer;
	for(vector<supervisor_msgs::ActionMS>::iterator it = actionList.begin(); it != actionList.end(); it++){
		if(action.name != it->name){
			continue;
		}
		if(action.parameters.size() == it->parameters.size()){
			vector<string>::iterator itp2 = action.parameters.begin();
			for(vector<string>::iterator itp = it->parameters.begin(); itp != it->parameters.end(); itp++){
				if(*itp != *itp2){
					continue;
				}
				itp2++;
			}
		}else{
			continue;
		}
		if(action.actors.size() == it->actors.size()){
			vector<string>::iterator ita2 = action.actors.begin();
			for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
				if(*ita != *ita2){
					continue;
				}
				ita2++;
			}
		}else{
			continue;
		}


		answer.first = true;
		answer.second = *it;
		return answer;
	}

	answer.first = false;
	return answer;
}

/*
Function which return the action (ActionMS format) corresponding to an id
	@id: the id of the action
*/
pair<bool, supervisor_msgs::ActionMS> MSManager::getActionFromId(int id){

	boost::unique_lock<boost::mutex> lock(actionList_mutex);
	pair<bool, supervisor_msgs::ActionMS> answer;
	if(id < actionId){
		answer.second = actionList[id];
		answer.first = true;
		return answer;
	}else{//the action does not exist
		ROS_ERROR("[mental_state] Unknown action id");
		answer.first = false;
		return answer;
	}
}

/*
Function which return the plan (PLanMS format) corresponding to an id
	@id: the id of the plan
*/
pair<bool, supervisor_msgs::PlanMS> MSManager::getPlanFromId(int id){

	boost::unique_lock<boost::mutex> lock(planList_mutex);
	pair<bool, supervisor_msgs::PlanMS> answer;
	if(id < planId){
		answer.second = planList[id];
		answer.first = true;
		return answer;
	}else{//the plan does not exist
		ROS_ERROR("[mental_state] Unknown plan id");
		answer.first = false;
		return answer;
	}
}



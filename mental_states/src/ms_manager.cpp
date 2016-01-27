/**
author Sandra Devin

Class which allows to maintain mental states of the agents
**/

#include <mental_states/ms_manager.h>

MSManager::MSManager(){
	actionId_ = 0;
	planId_ = 0;
}

vector<supervisor_msgs::ActionMS> MSManager::getActionList(){

   boost::unique_lock<boost::mutex> lock(actionListMutex_);
	return actionList_;
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
		string actorsTopic = "goals/";
		actorsTopic = actorsTopic + *it + "_actors";
		vector<string> actors;
		node.getParam(actorsTopic, actors);
		string objectiveTopic = "goals/";
		objectiveTopic = objectiveTopic + *it + "_objective";
		vector<string> objective;
		node.getParam(objectiveTopic, objective);
		//we transform the objective into facts
		vector<toaster_msgs::Fact> obj;
		for(vector<string>::iterator ob = objective.begin(); ob != objective.end(); ob++){
			int beg = ob->find(',');
    		int end = ob->find(',', beg+1);
			toaster_msgs::Fact fact;
    		fact.subjectId = ob->substr(0, beg);
    		fact.property = ob->substr(beg+1, end - beg - 1);
    		fact.propertyType = "state";
    		fact.targetId = ob->substr(end+1, ob->size() - end - 1);
			obj.push_back(fact);
		}
		//we then fill the goal list
		supervisor_msgs::GoalMS goal;
		goal.name = *it;
		goal.actors = actors;
		goal.objective = obj;
		goalList_.push_back(goal);
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
		string paramTopic = "highLevelActions/";
		paramTopic = paramTopic + *it + "_param";
		vector<string> params;
		node.getParam(paramTopic, params);
		string actorsTopic = "highLevelActions/";
		actorsTopic = actorsTopic + *it + "_actors";
		vector<string> actors;
		node.getParam(actorsTopic, actors);
		string precTopic = "highLevelActions/";
		precTopic = precTopic + *it + "_prec";
		vector<string> prec;
		node.getParam(precTopic, prec);
		//we transform the preconditions into facts
		vector<toaster_msgs::Fact> precFact;
		for(vector<string>::iterator p = prec.begin(); p != prec.end(); p++){
			int beg = p->find(',');
    		int end = p->find(',', beg+1);
			toaster_msgs::Fact fact;
    		fact.subjectId = p->substr(0, beg);
    		fact.property = p->substr(beg+2, end - beg - 2);
    		fact.propertyType = "state";
    		fact.targetId = p->substr(end+2, p->size() - end - 2);
			precFact.push_back(fact);
		}
		string effectsTopic = "highLevelActions/";
		effectsTopic = effectsTopic + *it + "_effects";
		vector<string> effects;
		node.getParam(effectsTopic, effects);
		//we transform the effects into facts
		vector<toaster_msgs::Fact> effectsFact;
		for(vector<string>::iterator e = effects.begin(); e != effects.end(); e++){
			int beg = e->find(',');
    		int end = e->find(',', beg+1);
			toaster_msgs::Fact fact;
    		fact.subjectId = e->substr(0, beg);
    		fact.property = e->substr(beg+2, end - beg - 2);
    		fact.propertyType = "state";
    		fact.targetId = e->substr(end+2, e->size() - end - 2);
			effectsFact.push_back(fact);
		}
		//we then fill the high level action list
		supervisor_msgs::ActionMS highLevelAction;
		highLevelAction.name = *it;
		highLevelAction.parameters = params;
		highLevelAction.actors = actors;
		highLevelAction.prec = precFact;
		highLevelAction.effects = effectsFact;
		highLevelActions_.push_back(highLevelAction);
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
	vector<supervisor_msgs::ActionMS> toDone;
	for(vector<supervisor_msgs::ActionMS>::iterator it = toCheckActions.begin(); it != toCheckActions.end(); it++){
		if(db.factsAreIn(agent, it->effects)){
			toDone.push_back(*it);
		}
	}

	//TODO: for all in PROGRESS action, if the agent see its actors and there are not perfoming it anymore, the agent considers the action DONE

	db.addActionsState(toDone, agent, "DONE");
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
		vector<supervisor_msgs::ActionMS> toNeeded;
		vector<supervisor_msgs::ActionMS> toReady;
		for(vector<supervisor_msgs::ActionMS>::iterator it = readyActions.begin(); it != readyActions.end(); it++){
			if(!db.factsAreIn(agent, it->prec)){ //if the preconditions are not respected anymore, the action goes back to NEEDED
				toNeeded.push_back(*it);
			}
		}

		//Check causaul links of PLANNED actions
		vector<int> plannedIds = db.getActionsIdFromState(agent, "PLANNED");
		vector<int> linksOkIds;
		vector<supervisor_msgs::ActionMS> linksOk;
		if(plannedIds.size() != 0){
			for(vector<int>::iterator it = plannedIds.begin(); it != plannedIds.end(); it++){
				vector<toaster_msgs::Fact> toCheck;
				//we get all actions id with a link to the action
				for(vector<supervisor_msgs::Link>::iterator itl = agentPlan.second.links.begin(); itl != agentPlan.second.links.end(); itl++){
					if(itl->following == *it){
						toaster_msgs::Fact fact;
						ostringstream toString;
						toString << itl->origin;
						fact.subjectId = toString.str();
						fact.property = "actionState";
    		         fact.propertyType = "state";
						fact.targetId = "DONE";
						toCheck.push_back(fact);
					}
				}
				//we check the agent thinks all these actions are DONE
				if(db.factsAreIn(agent, toCheck)){
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
				toReady.push_back(*it);
			}else{
				toNeeded.push_back(*it);
			}
		}
		for(vector<supervisor_msgs::ActionMS>::iterator it = neededActions.begin(); it != neededActions.end(); it++){
			if(db.factsAreIn(agent, it->prec)){ //if the preconditions are respected the action becomes READY, else it remains to NEEDED
				toReady.push_back(*it);
			}
		}

		db.addActionsState(toNeeded, agent, "NEEDED");
		db.addActionsState(toReady, agent, "READY");
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
				//TODO: if a human is not here and the robot has already tried to computes an other plan, it waits the human to come back
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

	for(vector<supervisor_msgs::ActionMS>::iterator it = highLevelActions_.begin(); it != highLevelActions_.end(); it++){
		if(it->name == name){
			return *it;
		}
	}

	ROS_ERROR("[mental_state] Unknown action name: %s", name.c_str());
	return action;

}

int MSManager::getAndIncreasePlanId(){
	planId_++;
	return (planId_ - 1);

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
	newAction.id = actionId_;
	actionId_ ++;
	newAction.parameters = action.parameters;
	newAction.actors = action.actors;
	for(vector<toaster_msgs::Fact>::iterator it = highLevelAction.prec.begin(); it != highLevelAction.prec.end(); it++){
		toaster_msgs::Fact fact;
		fact.subjectId = highLevelNames[it->subjectId];
		fact.targetId = highLevelNames[it->targetId];
		fact.property = it->property;
    	fact.propertyType = "state";
		newAction.prec.push_back(fact);
	}
	for(vector<toaster_msgs::Fact>::iterator it = highLevelAction.effects.begin(); it != highLevelAction.effects.end(); it++){
		toaster_msgs::Fact fact;
		fact.subjectId = highLevelNames[it->subjectId];
		fact.targetId = highLevelNames[it->targetId];
		fact.property = it->property;
    	fact.propertyType = "state";
		newAction.effects.push_back(fact);
	}

	//Add the action to the list of actions
	boost::unique_lock<boost::mutex> lock(actionListMutex_);
	actionList_.push_back(newAction);
	

	return newAction;

}

/*
Function which return a goal giving its name
	@name: name of the goal
*/
supervisor_msgs::GoalMS* MSManager::getGoalByName(string name){

	supervisor_msgs::GoalMS* goal = NULL;

	boost::unique_lock<boost::mutex> lock(goalListMutex_);
	
	for(vector<supervisor_msgs::GoalMS>::iterator it = goalList_.begin(); it != goalList_.end(); it++){
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

	boost::unique_lock<boost::mutex> lock(actionListMutex_);
	
	vector<supervisor_msgs::ActionMS> actions;
	for(vector<int>::iterator it = ids.begin(); it != ids.end(); it++){
		actions.push_back(actionList_[*it]);
	}
	
	return actions;
}

/*
Function which add a plan to the plan list
	@plan: the plan to add
*/
void MSManager::addPlanToList(supervisor_msgs::PlanMS plan){

	boost::unique_lock<boost::mutex> lock(planListMutex_);
	planList_.push_back(plan);
}

/*
Function which return the plan an agent think in PROGRESS (bool is at true if there is a plan, false otherwize)
	@agent: the agent name
*/
pair<bool, supervisor_msgs::PlanMS> MSManager::getAgentPlan(string agent){
	
	DBInterface db;
	int planId = db.getAgentIdPlan(agent);// we get the id of the plan
	pair<bool, supervisor_msgs::PlanMS> answer;
	boost::unique_lock<boost::mutex> lock(planListMutex_);
	//we return the corresponding plan
	if(planId != -1){
		answer.first = true;
		answer.second = planList_[planId];
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

	boost::unique_lock<boost::mutex> lock(actionListMutex_);
	pair<bool, supervisor_msgs::ActionMS> answer;
	for(vector<supervisor_msgs::ActionMS>::iterator it = actionList_.begin(); it != actionList_.end(); it++){
	   bool same = false;
		if(action.name != it->name){
			continue;
		}
		if(action.parameters.size() == it->parameters.size()){
			vector<string>::iterator itp2 = action.parameters.begin();
			for(vector<string>::iterator itp = it->parameters.begin(); itp != it->parameters.end(); itp++){
				if(*itp != *itp2){
					same = true;
					break;
				}
				itp2++;
			}
			if(same){
			   continue;
			}
		}else{
			continue;
		}
		if(action.actors.size() == it->actors.size()){
			vector<string>::iterator ita2 = action.actors.begin();
			for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
				if(*ita != *ita2){
					same = false;
					continue;
				}
				ita2++;
			}
			if(same){
			   continue;
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

	boost::unique_lock<boost::mutex> lock(actionListMutex_);
	pair<bool, supervisor_msgs::ActionMS> answer;
	if(id < actionId_){
		answer.second = actionList_[id];
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

	boost::unique_lock<boost::mutex> lock(planListMutex_);
	pair<bool, supervisor_msgs::PlanMS> answer;
	if(id < planId_){
		answer.second = planList_[id];
		answer.first = true;
		return answer;
	}else{//the plan does not exist
		ROS_ERROR("[mental_state] Unknown plan id");
		answer.first = false;
		return answer;
	}
}

/*
Function which convert an ActionMS in Action
	@actionMS: the action to convert
*/
supervisor_msgs::Action MSManager::convertActionMStoAction(supervisor_msgs::ActionMS actionMS){

	supervisor_msgs::Action action;
	action.name = actionMS.name;
	action.id = actionMS.id;
	action.parameters = actionMS.parameters;
	action.actors = actionMS.actors;

	return action;
}



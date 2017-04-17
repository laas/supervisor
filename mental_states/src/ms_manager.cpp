#include "mental_states/ms_manager.h"

/**
 * \brief Constructor of the class
 * */
MsManager::MsManager(ros::NodeHandle* node)
{
    //Initialize parameters
    node_ = node;
    node_->getParam("supervisor/robot/name", robotName_);
    node_->getParam("/supervisor/AgentX", agentX_);
    node_->getParam("/entities/agents", agents_);
    node_->getParam("/mental_states/nonObservableFacts_", nonObservableFacts_);

    //we do not compute mental states for the robot
    for(std::vector<std::string>::iterator it = agents_.begin(); it != agents_.end(); it++){
        if(*it == robotName_){
            agents_.erase(it);
            break;
        }
    }
    currentRobotAction_.id = -1;
    currentPlan_.id = -1;
    currentRobotGoal_= "NONE";

    //Initialize clients
    client_db_ = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");

    //Initialize high level names (from params)
    fillHighLevelNames();

    //Initialize mental states
    initMentalStates();

}

/**
 * \brief Fill the highLevelNames map from param
 * */
void MsManager::fillHighLevelNames(){

    std::vector<std::string> manipulableObjects;
    node_->getParam("/entities/objects", manipulableObjects);
    for(std::vector<std::string>::iterator it = manipulableObjects.begin(); it != manipulableObjects.end(); it++){
        highLevelNames_[*it] = *it;
        std::string paramName = "/entities/highLevelName/" + *it;
        if(node_->hasParam(paramName)){
            node_->getParam(paramName, highLevelRefinment_[*it]);
        }
    }

    std::vector<std::string> supportObjects;
    node_->getParam("/entities/supports", supportObjects);
    for(std::vector<std::string>::iterator it = supportObjects.begin(); it != supportObjects.end(); it++){
        highLevelNames_[*it] = *it;
        std::string paramName = "/entities/highLevelName/" + *it;
        if(node_->hasParam(paramName)){
            node_->getParam(paramName, highLevelRefinment_[*it]);
        }
    }

    std::vector<std::string> containerObjects;
    node_->getParam("/entities/containers", containerObjects);
    for(std::vector<std::string>::iterator it = containerObjects.begin(); it != containerObjects.end(); it++){
        highLevelNames_[*it] = *it;
        std::string paramName = "/entities/highLevelName/" + *it;
        if(node_->hasParam(paramName)){
            node_->getParam(paramName, highLevelRefinment_[*it]);
        }
    }

    for(std::map<std::string, std::vector<std::string> >::iterator it = highLevelRefinment_.begin(); it != highLevelRefinment_.end(); it++){
        for(std::vector<std::string>::iterator ith = it->second.begin(); ith != it->second.end(); ith++){
            highLevelNames_[*ith] = it->first;
        }
    }
}


/**
 * \brief Init the agents mental states
 * */
void MsManager::initMentalStates(){

    for(std::vector<std::string>::iterator it = agents_.begin(); it != agents_.end(); it++){
        supervisor_msgs::MentalState ms;
        ms.agentName = *it;
        ms.robotGoal = "NONE";
        ms.agentGoal = "NONE";
        ms.currentRobotAction.id = -1;
        msList_.push_back(ms);
    }
}

/**
 * \brief Check effects of todo and in progress actions
 * @return true if something changed
 * */
bool MsManager::checkEffects(){

    bool changed = false;

    for(std::vector<supervisor_msgs::MentalState>::iterator itms = msList_.begin(); itms != msList_.end(); itms++){
        //for all todo actions, we check if the agent can see the effects of the action
        for(std::vector<supervisor_msgs::Action>::iterator it = itms->todoActions.begin(); it != itms->todoActions.end(); it++){
            if(areFactsInTable(it->effects, itms->agentName, true)){
                //the agent deduces that the action has been performed
                itms->previousActions.push_back(*it);
                addEffects(*it, itms->agentName);
                itms->todoActions.erase(it);
                it--;
                changed = true;
            }
        }
        //for the robot current action:
        //if the agent sees the robot, he knows the action is in progress
        if(currentRobotAction_.id != -1 && itms->currentRobotAction.id == -1){
            if(canSee(itms->agentName, robotName_)){
                itms->currentRobotAction = currentRobotAction_;
            }
        }
        //if the agent sees the robot and the action is over, the agent knows the action is over
        if(currentRobotAction_.id == -1 && itms->currentRobotAction.id != -1){
            if(canSee(itms->agentName, robotName_)){
                addEffects(itms->currentRobotAction, itms->agentName);
                itms->currentRobotAction.succeed = true;
                itms->previousActions.push_back(itms->currentRobotAction);
                itms->currentRobotAction.id = -1;
            }
        }
    }

    return changed;
}

/**
 * \brief Check preconditions of planned actions
 * @return true if something changed
 * */
bool MsManager::checkPrecs(){

    if(currentPlan_.id == -1){
        return false;
    }

    bool changed = false;
    for(std::vector<supervisor_msgs::MentalState>::iterator itms = msList_.begin(); itms != msList_.end(); itms++){
        for(std::vector<supervisor_msgs::Action>::iterator it = itms->todoActions.begin(); it != itms->todoActions.end(); it++){
            //we check if the action has been executed
            bool executed = false;
            for(std::vector<supervisor_msgs::Action>::iterator itp = itms->previousActions.begin(); itp != itms->previousActions.end(); itp++){
                if(itp->id == it->id){
                    //if the action was the first half of an action we rename the remaining part
                    if(itp->name == "pick" && it->name != "pick"){
                        it->name = it->name.substr(7);//remove 'pickand' part of the name
                        itp->id = -1;//we change the id to not consider the action done the next loop
                    }else{
                        itms->todoActions.erase(it);
                        it--;
                        executed = true;
                    }
                    break;
                }
            }
            if(!executed){
                //check if the todo actions are still todo
                if(!isInList(it->actors, agentX_) && !areFactsInTable(it->precs, itms->agentName, true)){
                    //the action is not feasible anymore
                    itms->plannedActions.push_back(*it);
                    itms->todoActions.erase(it);
                    it--;
                }
            }
        }
        for(std::vector<supervisor_msgs::Action>::iterator it = itms->plannedActions.begin(); it != itms->plannedActions.end(); it++){
            //for the planned actions, we first check causal links
            bool linksOk = true;
            for(std::vector<supervisor_msgs::Link>::iterator itl = currentPlan_.links.begin(); itl != currentPlan_.links.end(); itl++){
                if(itl->following == it->id){
                    linksOk = false;
                    for(std::vector<supervisor_msgs::Action>::iterator itd = itms->previousActions.begin(); itd != itms->previousActions.end(); itd++){
                        if(itl->origin == itd->id && itd->succeed){
                            linksOk = true;
                            break;
                        }
                    }
                    if(!linksOk){
                        break;
                    }
                }
            }
            if(!linksOk){
                continue;
            }
            //then if the causal links are ok, we check preconditions
            if(areFactsInTable(it->precs, itms->agentName, true)){
                itms->todoActions.push_back(*it);
                itms->plannedActions.erase(it);
                it--;
            }
        }
    }

    return changed;
}


/**
 * \brief Check goals state
 * @return true if something changed
 * */
void MsManager::checkGoals(){

    for(std::vector<supervisor_msgs::MentalState>::iterator itms = msList_.begin(); itms != msList_.end(); itms++){
        if(itms->robotGoal != currentRobotGoal_ && canSee(itms->agentName, robotName_)){
            itms->robotGoal = currentRobotGoal_;
            std::string actorsParam = "goal_manager/goals/" + currentRobotGoal_ + "_actors";
            std::vector<std::string> actors;
            node_->getParam(actorsParam, actors);
            if(isInList(actors, itms->agentName)){
                itms->agentGoal = currentRobotGoal_;
            }
        }
    }

}


/**
 * \brief Look if facts are in an agent table
 * @param facts the facts to test
 * @param agent the name of the agent
 * @param highLevel true if we should consider high level objects
 * @return true if the facts are in the agent table
 * */
bool MsManager::areFactsInTable(std::vector<toaster_msgs::Fact> facts, std::string agent, bool highLevel){

    for(std::vector<toaster_msgs::DatabaseTable>::iterator ita = agentsTable_.begin(); ita != agentsTable_.end(); ita++){
        if(ita->agentName == agent){
            for(std::vector<toaster_msgs::Fact>::iterator itf = facts.begin(); itf != facts.end(); itf++){
                if(itf->subjectId == "NULL"){
                    for(std::vector<toaster_msgs::Fact>::iterator itt = ita->facts.begin(); itt != ita->facts.end(); itt++){
                        if(itt->property == itf->property &&
                                ((highLevelNames_[itt->targetId] == itf->targetId && highLevel)
                                 || itt->targetId == itf->targetId)){
                            return false;
                        }
                    }
                }else if(itf->targetId == "NULL"){
                    for(std::vector<toaster_msgs::Fact>::iterator itt = ita->facts.begin(); itt != ita->facts.end(); itt++){
                        if(itt->property == itf->property &&
                                ((highLevelNames_[itt->subjectId] == itf->subjectId && highLevel)
                                 || itt->subjectId == itf->subjectId)){
                            return false;
                        }
                    }
                }else{
                    bool find = false;
                    for(std::vector<toaster_msgs::Fact>::iterator itt = ita->facts.begin(); itt != ita->facts.end(); itt++){
                        if(itt->property == itf->property &&
                                ((highLevelNames_[itt->subjectId] == itf->subjectId  && highLevel)
                                 || itt->subjectId == itf->subjectId) &&
                                ((highLevelNames_[itt->targetId] == itf->targetId  && highLevel)
                                 || itt->targetId == itf->targetId)){
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

    return false;
}

/**
 * \brief Check if an agent can see another agent
 * @param agent the name of the agent who need to see
 * @param target the name of the agent to see
 * @return true if the agent can see the target
 * */
bool MsManager::canSee(std::string agent, std::string target){

    if(agent == target){
        return true;
    }

    toaster_msgs::Fact fact;
    fact.subjectId = target;
    fact.property = "isVisibleBy";
    fact.targetId = agent;
    std::vector<toaster_msgs::Fact> facts;
    facts.push_back(fact);

    return areFactsInTable(facts, robotName_, false);
}

/**
 * \brief Add the effects of an action into the table of an agent
 * @param action the action
 * @param agent the agent name
 * */
void MsManager::addEffects(supervisor_msgs::Action action, std::string agent){


    std::vector<toaster_msgs::Fact> toAdd, toRm;

    for(std::vector<toaster_msgs::Fact>::iterator it = action.effects.begin(); it != action.effects.end(); it++){
        if(isInList(nonObservableFacts_, it->property)){
            it->factObservability = 0.0;
        }else{
            it->factObservability = 1.0;
        }
        if(it->targetId == "NULL" || it->subjectId == "NULL"){
            toRm.push_back(*it);
        }else{
            toAdd.push_back(*it);
        }
    }

    toaster_msgs::SetInfoDB srv;
    srv.request.agentId = agent;
    srv.request.infoType = "FACT";
    if(toRm.size() > 0){
        srv.request.facts = toRm;
        srv.request.add = false;
        if (!client_db_.call(srv)){
         ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
        }
    }
    if(toAdd.size() > 0){
        srv.request.facts = toAdd;
        srv.request.add = true;
        if (!client_db_.call(srv)){
         ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
        }
    }
}

/**
 * \brief Check if a string element is in a list
 * @param list the list
 * @param element the element
 * @return true if the element is in the list
 * */
bool MsManager::isInList(std::vector<std::string> list, std::string element){

    for(std::vector<std::string>::iterator it = list.begin(); it != list.end(); it++){
        if(*it == element){
            return true;
        }
    }

    return false;
}

/**
 * \brief Add the preconditions and effects to an action
 * @param action the initial action
 * @return the action with preconditions and effects
 * */
supervisor_msgs::Action MsManager::addPrecsAndEffects(supervisor_msgs::Action action){

    supervisor_msgs::Action toReturn;
    toReturn = action;

    //we get the highLevel precs and effects from param
    std::string precsTopic = "highLevelActions/"+ action.name + "_prec";
    std::string effectsTopic = "highLevelActions/"+ action.name + "_effects";
    std::vector<std::string> stringPrecs, stringEffects;
    node_->getParam(precsTopic, stringPrecs);
    node_->getParam(effectsTopic, stringEffects);

    //we convert them into facts
    std::vector<toaster_msgs::Fact> highLevelPrecs, highLevelEffects;
    for(std::vector<std::string>::iterator it = stringPrecs.begin(); it != stringPrecs.end(); it++){
        int beg = it->find(',');
        int end = it->find(',', beg+1);
        toaster_msgs::Fact fact;
        fact.subjectId = it->substr(0, beg);
        fact.property = it->substr(beg+2, end - beg - 2);
        fact.propertyType = "state";
        fact.targetId = it->substr(end+2, it->size() - end - 2);
        highLevelPrecs.push_back(fact);
    }
    for(std::vector<std::string>::iterator it = stringEffects.begin(); it != stringEffects.end(); it++){
        int beg = it->find(',');
        int end = it->find(',', beg+1);
        toaster_msgs::Fact fact;
        fact.subjectId = it->substr(0, beg);
        fact.property = it->substr(beg+2, end - beg - 2);
        fact.propertyType = "state";
        fact.targetId = it->substr(end+2, it->size() - end - 2);
        highLevelEffects.push_back(fact);
    }

    //we replace the name of the param
    std::vector<toaster_msgs::Fact> precs, effects;
    for(std::vector<toaster_msgs::Fact>::iterator it = highLevelPrecs.begin(); it != highLevelPrecs.end(); it++){
        toaster_msgs::Fact fact;
        fact.property = it->property;
        fact.propertyType = it->propertyType;
        if(it->subjectId == "mainAgent"){
            fact.subjectId = action.actors[0];
        }else if(it->subjectId == "NULL" || it->subjectId == "true" || it->subjectId == "false"){
            fact.subjectId = it->subjectId;
        }else{
            for(int i = 0; i < action.parameter_keys.size(); i++){
                if(action.parameter_keys[i] == it->subjectId){
                    fact.subjectId = action.parameter_values[i];
                    break;
                }
            }
        }
        if(it->targetId == "mainAgent"){
            fact.targetId = action.actors[0];
        }else if(it->targetId == "NULL" || it->targetId == "true" || it->targetId == "false"){
            fact.targetId = it->targetId;
        }else{
            for(int i = 0; i < action.parameter_keys.size(); i++){
                if(action.parameter_keys[i] == it->targetId){
                    fact.targetId = action.parameter_values[i];
                    break;
                }
            }
        }
        precs.push_back(fact);
    }
    for(std::vector<toaster_msgs::Fact>::iterator it = highLevelEffects.begin(); it != highLevelEffects.end(); it++){
        toaster_msgs::Fact fact;
        fact.property = it->property;
        fact.propertyType = it->propertyType;
        if(it->subjectId == "mainAgent"){
            fact.subjectId = action.actors[0];
        }else if(it->subjectId == "NULL" || it->subjectId == "true" || it->subjectId == "false"){
            fact.subjectId = it->subjectId;
        }else{
            for(int i = 0; i < action.parameter_keys.size(); i++){
                if(action.parameter_keys[i] == it->subjectId){
                    fact.subjectId = action.parameter_values[i];
                    break;
                }
            }
        }
        if(it->targetId == "mainAgent"){
            fact.targetId = action.actors[0];
        }else if(it->targetId == "NULL" || it->targetId == "true" || it->targetId == "false"){
            fact.targetId = it->targetId;
        }else{
            for(int i = 0; i < action.parameter_keys.size(); i++){
                if(action.parameter_keys[i] == it->targetId){
                    fact.targetId = action.parameter_values[i];
                    break;
                }
            }
        }
        effects.push_back(fact);
    }

    //we add precs and effects to the action
    toReturn.precs = precs;
    toReturn.effects = effects;

    return toReturn;
}


/**
 * \brief Say if an action is in a list (considering high level objects)
 * @param action the looking action
 * @param actions the list
 * @return the corresponding action of the list (action with id = -1 if the action is not in the list)
 * */
supervisor_msgs::Action MsManager::isInList(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actions){

    bool hasAgentXAction = false;
    bool hasAction = false;
    supervisor_msgs::Action toReturn;
    for(std::vector<supervisor_msgs::Action>::iterator it = actions.begin(); it != actions.end(); it++){
        bool find = true;
        if(action.name == it->name || (action.name == "pick" && boost::contains(it->name, "pick"))){
            if(action.name == "pick" && it->name != "pick"){
                //we only check the object
                std::string object;
                for(int i = 0; i < action.parameter_keys.size(); i++){
                    if(action.parameter_keys[i] == "object"){
                        object = action.parameter_values[i];
                        break;
                    }
                }
                for(int i = 0; i < it->parameter_keys.size(); i++){
                    if(it->parameter_keys[i] == "object"){
                        if(highLevelNames_[it->parameter_values[i]] != highLevelNames_[object]){
                            find = false;
                        }
                    }
                }
            }else{
                if(action.parameter_values.size() != it->parameter_values.size()){
                    find = false;
                }else{
                    for(int i = 0; i < action.parameter_values.size(); i++){
                        if(highLevelNames_[action.parameter_values[i]] != highLevelNames_[it->parameter_values[i]]){
                            find = false;
                            break;
                        }
                    }
                }
            }
        }else{
            find = false;
        }
        if(find){
            //we keep in priority actions from the actor, then agentX actions, then others
            if(action.actors[0] == it->actors[0]){
                return *it;
            }else if(!hasAgentXAction && it->actors[0] == agentX_){
                hasAgentXAction = true;
                hasAction = true;
                toReturn = *it;
            }
        }
    }

    if(!hasAction){
        toReturn.id = -1;
    }

    return toReturn;
}

/**
 * \brief Add or remove a fact in an agent table
 * @param fact the fact to add
 * @param agent the agent
 * @param add true if the fact needs to be add, false for removing
 * */
void MsManager::addRmFactToAgent(toaster_msgs::Fact fact, std::string agent, bool add){

    if(isInList(nonObservableFacts_, fact.property)){
        fact.factObservability = 0.0;
    }else{
        fact.factObservability = 1.0;
    }

    toaster_msgs::SetInfoDB srv;
    srv.request.agentId = agent;
    std::vector<toaster_msgs::Fact> facts;
    facts.push_back(fact);
    srv.request.facts = facts;
    srv.request.infoType = "FACT";
    srv.request.add = add;
    if (!client_db_.call(srv)){
     ROS_ERROR("[mental_state] Failed to call service database_manager/set_info");
    }
}

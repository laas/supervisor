/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "std_srvs/Trigger.h"

#include "toaster_msgs/FactList.h"
#include "toaster_msgs/GetInfoDB.h"
#include "toaster_msgs/ExecuteDB.h"

#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/Ask.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/GiveInfo.h"
#include "supervisor_msgs/MentalStatesList.h"
#include "supervisor_msgs/GoalsList.h"
#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/SharedPlan.h"

typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;

ros::NodeHandle* node_;
ros::ServiceClient* client_end_plan_;
ros::ServiceClient* client_ask_;
ros::ServiceClient* client_inform_;
ros::ServiceClient* client_execute_db_;
ros::ServiceClient* client_get_info_db_;
std::string robotName_, robotState_, xAgent_, mainPartner_;
Client* actionClient_;
supervisor_msgs::Action currentAction_;
std::map<std::string, std::string> highLevelNames_;
std::map<std::string, std::vector<std::string> > highLevelRefinment_;
std::string mode_;
bool timerStarted_;
ros::Time start_;
double timeAdaptation_, timeWaitHuman_;
supervisor_msgs::Action previousManagedAction_, previousTalkedHumanAction_, previousHumanAction_;
std::string currentGoal_;
std::vector<toaster_msgs::Fact> areaFacts_;
std::string areaInform_, areaIdle_;
std::vector<supervisor_msgs::Action> actionsTodo_, previousActions_;
supervisor_msgs::SharedPlan currentPlan_;
bool shouldRetractRight_, shouldRetractLeft_;
std::string rightRestPosition_, leftRestPosition_;
int planPrevId_, msPrevId_;
bool isGivingInfo_;
bool hasActed_;
int prevRobotId_;
bool alreadyReplan_;

/**
 * \brief Fill the highLevelNames map from param
 * */
void fillHighLevelNames(){

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

    std::vector<std::string> agents;
    node_->getParam("/entities/agents", agents);
    for(std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); it++){
        highLevelNames_[*it] = *it;
    }

    for(std::map<std::string, std::vector<std::string> >::iterator it = highLevelRefinment_.begin(); it != highLevelRefinment_.end(); it++){
        for(std::vector<std::string>::iterator ith = it->second.begin(); ith != it->second.end(); ith++){
            highLevelNames_[*ith] = it->first;
        }
    }
}



/**
 * \brief Function which return te object to lock given an action
 * @param action the attributed action
 * @return the object to lock
 * */
std::string getLockedObject(supervisor_msgs::Action action){

    std::string object;

    if(action.name == "pick" || action.name == "pickandplace" || action.name == "pickandplacereachable" || action.name == "pickanddrop"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                object = action.parameter_values[i];
            }
        }
    }else if(action.name == "place"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "support"){
                object = action.parameter_values[i];
            }
        }
    }else if(action.name == "drop"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "container"){
                object = action.parameter_values[i];
            }
        }
    }
 
    if(highLevelRefinment_[object].size() > 0){
        //need to refine the object
        std::vector<toaster_msgs::Fact> toTest;
        for(std::vector<std::string>::iterator it = highLevelRefinment_[object].begin(); it != highLevelRefinment_[object].end(); it++){
                toaster_msgs::Fact fact;
                fact.subjectId = *it;
                fact.property = "isReachableBy";
                fact.targetId = action.actors[0];
                toTest.push_back(fact);
        }
        std::vector<std::string> res;
        toaster_msgs::ExecuteDB srv;
        srv.request.command = "ARE_IN_TABLE";
        srv.request.type = "INDIV";
        srv.request.agent = robotName_;
        srv.request.facts = toTest;
        ros::ServiceClient client_db_execute_ = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
        if (client_db_execute_.call(srv)){
          std::vector<std::string> res = srv.response.results;
          for(int i = 0; i < res.size(); i++){
                if(res[i] == "true"){
                        return highLevelRefinment_[object][i];
                }
          }
        }else{
         ROS_ERROR("[action_executor] Failed to call service database_manager/execute");
        }
    }

    return object;

}

/**
 * \brief Says if an action as a "twin" in the todo list
 * @param action the action we are testing
 * @param actionsTodo list of the todo actions
 * @return true if the action is an identical action
 * */
bool isIdendicalAction(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actionsTodo){

    for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo.begin(); it != actionsTodo.end(); it++){
        if(action.id == it->id){
            continue;
        }
        bool find = true;
        if(action.name != it->name){
            find = false;
        }else if(it->actors[0] != xAgent_){
            //this process applies only for agent x actions
            find = false;
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
        if(find){
            return true;
        }
    }

    return false;

}

/**
 * \brief Callback for the end of the robot action
 * @param state the state of the action server
 * @param result the result of the action server
 * */
void actionDone(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result){

    robotState_ = "IDLE";
    shouldRetractRight_ = result->shouldRetractRight;
    shouldRetractLeft_ = result->shouldRetractLeft;
}



/**
 * \brief Says if an agent is in an area
 * @param agent the tested agent
 * @param area the tested area
 * @return true if the agent is in the area
 * */
bool isInArea(std::string agent, std::string area){

    for(std::vector<toaster_msgs::Fact>::iterator it = areaFacts_.begin(); it != areaFacts_.end(); it++){
        if(it->property == "IsInArea" && it->subjectId == agent && it->targetId == area){
            return true;
        }
    }

    return false;
}


/**
 * \brief Check if two actions are the same
 * @param action1 first action to compare
 * @param action2 second action to compare
 * */
bool areActionsEqual(supervisor_msgs::Action action1, supervisor_msgs::Action action2){

    if(action1.id == action2.id){
        return true;
    }

    //first check names
    if(action1.name != action2.name){
        return false;
    }

    //the check actors
    if(action1.actors.size() != action2.actors.size()){
        return false;
    }else{
        for(int i = 0; i < action2.actors.size(); i++){
            if(action1.actors[i] != action2.actors[i] && action1.actors[i] != "AGENTX" && action2.actors[i] != "AGENTX"){
                return false;
            }
        }
    }

    //then check params
    if(action1.parameter_values.size() != action2.parameter_values.size()){
        return false;
    }else{
        for(int i = 0; i < action1.parameter_values.size(); i++){
            if(action1.parameter_values[i] != action2.parameter_values[i] && highLevelNames_[action1.parameter_values[i]] != action2.parameter_values[i] && action1.parameter_values[i] != highLevelNames_[action2.parameter_values[i]]){
		return false;
            }
        }
    }

    return true;
}



/**
 * \brief Callback for todo actions
 * @param msg topic msg
 * */
void todoCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    planPrevId_ = msg->prevId;
    actionsTodo_ = msg->actions;

    if(planPrevId_ == prevRobotId_ && prevRobotId_ != -1){
	hasActed_ = true;
        prevRobotId_ = -1;
    }

    bool hasRobotAction = false;

    if(robotState_ == "IDLE" && actionsTodo_.size() > 0 && planPrevId_ == msPrevId_){
        bool isWaiting = false;

        supervisor_msgs::Action action;
        bool hasXAction = false;
        //we look for actions with the robot has actor
        for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo_.begin(); it != actionsTodo_.end(); it++){
            if(it->actors[0] == robotName_){
                hasRobotAction = true;
                action = *it;
                break;
            }else if(!hasXAction && it->actors[0] == xAgent_){
                //we remember the action in case we do not find a robot action
                hasXAction = true;
                action = *it;
            }
        }
        if(areActionsEqual(action, previousManagedAction_) && ((hasXAction & !timerStarted_) || hasRobotAction)){
            //we wait for the todo list update
            return;
       }

	supervisor_msgs::Action humanAction;
    bool hasHumanAction = false;
    for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo_.begin(); it != actionsTodo_.end(); it++){
        if(it->actors[0] == mainPartner_ &&  it->id != 0){
            //we remember the action to inform the human
            hasHumanAction = true;
            humanAction = *it;
            break;
        }
    }
    if(areActionsEqual(humanAction, previousTalkedHumanAction_)){
        //we already informed about this action
        hasHumanAction = false;
    }
           if(!areActionsEqual(humanAction, previousHumanAction_) && hasHumanAction){
        previousHumanAction_ = humanAction;
        alreadyReplan_ = false;
    }


        previousManagedAction_ = action;
        if(hasRobotAction){
            //we execute the action
            supervisor_msgs::ActionExecutorGoal goal;
            action.actors[0] = robotName_;
            goal.action = action;
            actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
            robotState_ = "ACTING";
            currentAction_ = action;
            hasActed_ = false;
            prevRobotId_ = action.id;
        }else if(hasXAction){
            //we try to attribute the action
            /** @todo implement priority for identical actions*/
            if(isIdendicalAction(action, actionsTodo_)){
                //if there is more than once this action to attribute we execute the action
                supervisor_msgs::ActionExecutorGoal goal;
                action.actors[0] = robotName_;
                goal.action = action;
                actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                robotState_ = "ACTING";
                currentAction_ = action;
                hasActed_ = false;
                prevRobotId_ = action.id;

                supervisor_msgs::EndPlan srv;
                srv.request.success = false;
                srv.request.evaluate = true;
                srv.request.objectLocked = getLockedObject(action);
                srv.request.agentLocked = robotName_;
                if (!client_end_plan_->call(srv)){
                   ROS_ERROR("[robot_decision] Failed to call service plan_elaboration/end_plan");
                }
            }else{
                //we try to attribute the action
                std::vector<std::string> possibleActors;
                if(timerStarted_ || isInArea(mainPartner_, areaIdle_)){
                    possibleActors.push_back(mainPartner_);
                }
                possibleActors.push_back(robotName_);
                if(possibleActors.size() == 1){
                    supervisor_msgs::EndPlan srv;
                    srv.request.success = false;
                    srv.request.evaluate = true;
                    srv.request.objectLocked = getLockedObject(action);
                    srv.request.agentLocked = possibleActors[0];
                    if (!client_end_plan_->call(srv)){
                       ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                    }
                    if(possibleActors[0] == robotName_){
                        //and we execute the action
                        supervisor_msgs::ActionExecutorGoal goal;
                        action.actors[0] = robotName_;
                        goal.action = action;
                        actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                        robotState_ = "ACTING";
                        currentAction_ = action;
                        hasActed_ = false;
                        prevRobotId_ = action.id;

                        supervisor_msgs::EndPlan srv;
                        srv.request.success = false;
                        srv.request.evaluate = true;
                        srv.request.objectLocked = getLockedObject(action);
                        srv.request.agentLocked = robotName_;
                        if (!client_end_plan_->call(srv)){
                           ROS_ERROR("[robot_decision] Failed to call service plan_elaboration/end_plan");
                        }
                    }
                }else if(possibleActors.size() > 1){
                    if(mode_ == "negotiation"){
                        supervisor_msgs::Ask srv_ask;
                        srv_ask.request.type = "ACTION";
                        srv_ask.request.subType = "WANT";
                        srv_ask.request.action = action;
                        srv_ask.request.receiver = mainPartner_;
                        srv_ask.request.waitForAnswer = true;
                        if (client_ask_->call(srv_ask)){
                          if(srv_ask.response.boolAnswer){
                              //the partner wants to perform the action
                              ROS_ERROR("end plan from neg yes!");
                              supervisor_msgs::EndPlan srv;
                              srv.request.success = false;
                              srv.request.evaluate = true;
                              srv.request.objectLocked = getLockedObject(action);
                              srv.request.agentLocked = mainPartner_;
                              if (!client_end_plan_->call(srv)){
                                 ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                              }
                          }else{
                              //the partner does not want to perform the action: the robot does it
                              supervisor_msgs::ActionExecutorGoal goal;
                              action.actors[0] = robotName_;
                              goal.action = action;
                              actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                              robotState_ = "ACTING";
                              currentAction_ = action;
                              hasActed_ = false;
                              prevRobotId_ = action.id;
                              supervisor_msgs::EndPlan srv;
                              srv.request.success = false;
                              srv.request.evaluate = true;
                              srv.request.objectLocked = getLockedObject(action);
                              srv.request.agentLocked = robotName_;
                              if (!client_end_plan_->call(srv)){
                                 ROS_ERROR("[robot_decision] Failed to call service plan_elaboration/end_plan");
                              }
                          }
                        }else{
                          ROS_ERROR("Failed to call service dialogue_node/ask");
                        }
                    }else if(mode_ == "adaptation"){
                        //we wait few time to see if the partner performs the action, if not the robot does it
                        isWaiting = true;
                        if(!timerStarted_){
                            start_ = ros::Time::now();
                            timerStarted_ = true;
                        }else{
                            ros::Duration d = ros::Time::now() - start_;
                            double duration = d.toSec();
                            if(duration >= timeAdaptation_){
                                //the partner does not perform it, the robot does it
                                supervisor_msgs::ActionExecutorGoal goal;
                                action.actors[0] = robotName_;
                                goal.action = action;
                                actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                                robotState_ = "ACTING";
                                currentAction_ = action;
                                supervisor_msgs::EndPlan srv;
                                srv.request.success = false;
                                srv.request.evaluate = true;
                                srv.request.objectLocked = getLockedObject(action);
                                srv.request.agentLocked = robotName_;
                                if (!client_end_plan_->call(srv)){
                                   ROS_ERROR("[robot_decision] Failed to call service plan_elaboration/end_plan");
                                }
                                timerStarted_ = false;
                            }
                        }
                    }else{
                        ROS_WARN("[robot_decision] Actual mode is: %s, it should be negotiation or adaptation", mode_.c_str());
                    }
                }
            }
        }else{
            //there is only actions to do for the human
            isWaiting = true;
            if(!timerStarted_){
                start_ = ros::Time::now();
                timerStarted_ = true;
            }else{
                ros::Duration d = ros::Time::now() - start_;
                double duration = d.toSec();
                if(duration >= timeWaitHuman_){
                     if(!alreadyReplan_){
                        if(isInArea(mainPartner_, areaInform_) && hasHumanAction){
		                    timerStarted_ = false;
							supervisor_msgs::GiveInfo srv;
		                    srv.request.type = "ACTION";
		                     srv.request.action = humanAction;
		                    srv.request.actionState = "SHOULD";
		                    srv.request.partner = mainPartner_;
		                    if (!client_inform_->call(srv)){
		                       ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
		                    }
                        }
                        alreadyReplan_ = true;
                   }else{
                        if(isInArea(mainPartner_, areaInform_)){
		                    timerStarted_ = false;
							alreadyReplan_ = false;
		                    supervisor_msgs::EndPlan srv;
		                    srv.request.success = false;
		                    srv.request.evaluate = false;
		                    srv.request.forgiveAction = true;
		                    srv.request.objectLocked = getLockedObject(actionsTodo_[0]);
		                    srv.request.agentLocked = mainPartner_;
		                    if (!client_end_plan_->call(srv)){
		                       ROS_ERROR("[robot_decision] Failed to call service plan_elaboration/end_plan");
		                    }
                        }
                  }
                }
            }
        }

        if(!isWaiting){
            timerStarted_ = false;
        }
    }

    if(robotState_ == "IDLE" && (actionsTodo_.size() == 0 || timerStarted_ || hasRobotAction)){
        //we retract the arms if needed
        if(shouldRetractRight_){
            supervisor_msgs::Action retractAction;
            supervisor_msgs::ActionExecutorGoal goal;
            retractAction.actors.push_back(robotName_);
            retractAction.name = "moveTo";
            retractAction.id = -1;
            retractAction.parameter_keys.push_back("arm");
            retractAction.parameter_values.push_back("right");
            retractAction.parameter_keys.push_back("position");
            retractAction.parameter_values.push_back(rightRestPosition_);
            goal.action = retractAction;
            actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
            robotState_ = "RETRACTING";
        }else if(shouldRetractLeft_){
            supervisor_msgs::Action retractAction;
            supervisor_msgs::ActionExecutorGoal goal;
            retractAction.actors.push_back(robotName_);
            retractAction.name = "moveTo";
            retractAction.id = -1;
            retractAction.parameter_keys.push_back("arm");
            retractAction.parameter_values.push_back("left");
            retractAction.parameter_keys.push_back("position");
            retractAction.parameter_values.push_back(leftRestPosition_);
            goal.action = retractAction;
            actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
            robotState_ = "RETRACTING";
        }
    }


}

/**
 * \brief Service call when the current robot action needs to be stopped
 * @param req the request of the service
 * @param res answer of the service
 * @return true
 * */
bool stopSrv(std_srvs::Trigger ::Request  &req, std_srvs::Trigger ::Response &res){

    if(robotState_ == "IDLE"){
        //stop the action
        actionClient_->cancelGoal();
        robotState_ = "STOPING";
        while(robotState_ == "STOPING"){
            ros::spinOnce();
        }
        res.success = true;
    }else{
        res.success = true;
    }

    return true;
}

/**
 * \brief Says if an element is in a list
 * @param element the tested element
 * @param list the tested list
 * @return true if the element is in the list
 * */
bool isInList(std::string element, std::vector<std::string> list){

    for(std::vector<std::string>::iterator it = list.begin(); it != list.end(); it++){
        if(*it == element){
            return true;
        }
    }

    return false;
}


/**
 * \brief Add the preconditions to an action
 * @param action the initial action
 * @return the action with preconditions
 * */
supervisor_msgs::Action addPrecs(supervisor_msgs::Action action){

    supervisor_msgs::Action toReturn;
    toReturn = action;

    //we get the highLevel precs and effects from param
    std::string precsTopic = "highLevelActions/"+ action.name + "_prec";
    std::vector<std::string> stringPrecs, stringEffects;
    node_->getParam(precsTopic, stringPrecs);

    //we convert them into facts
    std::vector<toaster_msgs::Fact> highLevelPrecs;
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

    //we replace the name of the param
    std::vector<toaster_msgs::Fact> precs, effects;
    for(std::vector<toaster_msgs::Fact>::iterator it = highLevelPrecs.begin(); it != highLevelPrecs.end(); it++){
        toaster_msgs::Fact fact;
        fact.property = it->property;
        fact.propertyType = it->propertyType;
        if(it->subjectId == "mainAgent"){
            fact.subjectId = action.actors[0];
        }else if(it->subjectId == "true"){
			fact.subjectId == "true";
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
        }else if(it->targetId == "true"){
                fact.targetId == "true";
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

    //we add precs and effects to the action
    toReturn.precs = precs;

    return toReturn;
}

/**
 * \brief Check individually if facts are in an agent table
 * @param facts the tested elements
 * @param agent the name of the agent
 * @return for each fact true if it is in the agent table
 * */
std::vector<bool> areFactsInTable(std::vector<toaster_msgs::Fact> facts, std::string agent){

    std::vector<bool> toReturn;

    toaster_msgs::GetInfoDB srv;
    srv.request.type = "FACT";
    srv.request.subType = "CURRENT";
    srv.request.agentId = agent;
    if (client_get_info_db_->call(srv)){

        std::vector<toaster_msgs::Fact> agentFacts = srv.response.resFactList.factList;

        for(std::vector<toaster_msgs::Fact>::iterator itf = facts.begin(); itf != facts.end(); itf++){
            if(itf->targetId == xAgent_ || itf->subjectId == xAgent_){
                toReturn.push_back(true);
                continue;
            }
            bool find = false;
            for(std::vector<toaster_msgs::Fact>::iterator ita = agentFacts.begin(); ita != agentFacts.end(); ita++){
                if(itf->property == ita->property && (itf->subjectId == highLevelNames_[ita->subjectId] || itf->subjectId == ita->subjectId)
                        && (itf->targetId == highLevelNames_[ita->targetId] || itf->targetId == ita->targetId)){
                    find = true;
                    break;
                }
            }
            toReturn.push_back(find);
        }

        return toReturn;
    }else{
        ROS_ERROR("[robot_decision] Failed to call service database_manager/execute");
    }

    return toReturn;
}

/**
 * \brief Solve a divergence of belief concerning an action
 * @param agent the agent concerned
 * @param ms the mental state of the concerned agent
 * @param action the action concerned
 * @param todo true if the action is in the robot todo list
 * */
void solveDB(std::string agent, supervisor_msgs::MentalState ms, supervisor_msgs::Action action, bool todo){


    if(todo){
        //we first check the causal links
        for(std::vector<supervisor_msgs::Link>::iterator it = currentPlan_.links.begin(); it != currentPlan_.links.end(); it++){
            if(it->following == action.id){
                bool find = false;
                //find origin action
                supervisor_msgs::Action originAction;
                for(std::vector<supervisor_msgs::Action>::iterator ita = currentPlan_.actions.begin(); ita != currentPlan_.actions.end(); ita++){
                    if(ita->id == it->origin){
                        originAction = *ita;
                        break;
                    }
                }
                for(std::vector<supervisor_msgs::Action>::iterator itp = ms.previousActions.begin(); itp != ms.previousActions.end(); itp++){
                    if(areActionsEqual(*itp, originAction) && itp->succeed){
                        find = true;
                        break;
                    }
                }
                if(!find){
                    //we find the missing info
                    supervisor_msgs::GiveInfo srv;
                    srv.request.type = "ACTION";
                    srv.request.action = originAction;
                    srv.request.actionState = "DONE";
                    srv.request.partner = agent;
                    if (!client_inform_->call(srv)){
                       ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
                    }
                    isGivingInfo_ = true;
                    return;
                }
            }
        }
        //then we check the preconditions
        supervisor_msgs::Action actionWithPrec = addPrecs(action);
        std::vector<bool> inTable = areFactsInTable(actionWithPrec.precs, agent);
        for(int i = 0; i < inTable.size(); i++){
            if(!inTable[i]){
                //we find the missing info
                supervisor_msgs::GiveInfo srv;
                srv.request.type = "FACT";
                srv.request.fact = actionWithPrec.precs[i];
                srv.request.isTrue = true;
                srv.request.partner = agent;
                if (!client_inform_->call(srv)){
                   ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
                }
                isGivingInfo_ = true;
                return;
            }
        }
    }else{
        //we first look if the action is considered donne for the robot
        for(std::vector<supervisor_msgs::Action>::iterator itp = previousActions_.begin(); itp != previousActions_.end(); itp++){
            if(areActionsEqual(*itp, action)){
                //we find the missing info
                supervisor_msgs::GiveInfo srv;
                srv.request.type = "ACTION";
                srv.request.action = action;
                if(itp->succeed){
                    srv.request.actionState = "DONE";
                }else{
                    srv.request.actionState = "FAILED";
                }
                srv.request.partner = agent;
                if (!client_inform_->call(srv)){
                   ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
                }
                isGivingInfo_ = true;
                return;
            }
        }

        //else we check the causal links
        for(std::vector<supervisor_msgs::Link>::iterator it = currentPlan_.links.begin(); it != currentPlan_.links.end(); it++){
            if(it->following == action.id){
                bool find = false;
                bool failed = false;
                supervisor_msgs::Action toInformAction;
                //find origin action
                supervisor_msgs::Action originAction;
                for(std::vector<supervisor_msgs::Action>::iterator ita = currentPlan_.actions.begin(); ita != currentPlan_.actions.end(); ita++){
                    if(ita->id == it->origin){
                        originAction = *ita;
                        break;
                    }
                }
                for(std::vector<supervisor_msgs::Action>::iterator itp = previousActions_.begin(); itp != previousActions_.end(); itp++){
                    if(areActionsEqual(*itp, originAction) && itp->succeed){
                        find = true;
                        break;
                    }else if(areActionsEqual(*itp, originAction) && !itp->succeed){
                        failed = true;
                        toInformAction = *itp;
                    }
                }
                if(!find){
                    supervisor_msgs::GiveInfo srv;
                    //the action is not consiered achieved in the robot knowledge
                    if(failed){
                        srv.request.actionState = "FAILED";
                    }else{
                        //we look for the exact action
                        for(std::vector<supervisor_msgs::Action>::iterator itp = ms.previousActions.begin(); itp != ms.previousActions.end(); itp++){
                            if(itp->id == it->origin){
                                toInformAction = *itp;
                                break;
                            }
                        }
                        srv.request.actionState = "NOT_PERFORMED";
                    }
                    srv.request.type = "ACTION";
                    srv.request.action = toInformAction;
                    srv.request.partner = agent;
                    if (!client_inform_->call(srv)){
                       ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
                    }
                    isGivingInfo_ = true;
                    return;
                }
            }
        }

        //then we check the preconditions
        supervisor_msgs::Action actionWithPrec = addPrecs(action);
        std::vector<bool> inTable = areFactsInTable(actionWithPrec.precs, robotName_);
        for(int i = 0; i < inTable.size(); i++){
            if(!inTable[i]){
                //we find the missing info
                supervisor_msgs::GiveInfo srv;
                srv.request.type = "FACT";
                srv.request.fact = actionWithPrec.precs[i];
                srv.request.isTrue = false;
                srv.request.partner = agent;
                if (!client_inform_->call(srv)){
                   ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
                }
                isGivingInfo_ = true;
                return;
            }
        }
    }
}

/**
 * \brief Callback for the area facts list topic
 * @param msg topic msg
 * */
void areaFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){

    areaFacts_ = msg->factList;

}

/**
 * \brief Callback for the goals list topic
 * @param msg topic msg
 * */
void goalCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    currentGoal_ = msg->currentGoal;
}

/**
 * \brief Callback for the previous actions list topic
 * @param msg topic msg
 * */
void previousCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    previousActions_ = msg->actions;
}

/**
 * \brief Callback for the shared plan topic
 * @param msg topic msg
 * */
void planCallback(const supervisor_msgs::SharedPlan::ConstPtr& msg){

    currentPlan_ = *msg;
}

/**
 * \brief Callback for the mental states
 * @param msg topic msg
 * */
void msCallback(const supervisor_msgs::MentalStatesList::ConstPtr& msg){

    msPrevId_ = msg->prevId;

    if(msg->infoGiven){
        isGivingInfo_ = false;
    }

    if(isGivingInfo_ == false && msg->changed && actionsTodo_.size() > 0 && msPrevId_ == planPrevId_){
        std::vector<supervisor_msgs::MentalState> ms = msg->mentalStates;
        for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms.begin(); itms != ms.end(); itms++){
            //we inform only if the human is here
            if(isInArea(itms->agentName, areaInform_)){
                //we check beliefs on the robot goal
                if(itms->robotGoal != currentGoal_){
                    supervisor_msgs::GiveInfo srv;
                    srv.request.type = "GOAL";
                    srv.request.goal = currentGoal_;
                    srv.request.partner = itms->agentName;
                    if (!client_inform_->call(srv)){
                       ROS_ERROR("[robot_decision] Failed to call service dialogue_node/give_info");
                    }
                }
                //we check if the actions todo for the human are the same
                bool actionTodo = false;
                for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo_.begin(); it != actionsTodo_.end(); it++){
                    //we check only human actions
                    if(isInList(itms->agentName, it->actors)){
                        actionTodo = true;
                        bool find = false;
                        for(std::vector<supervisor_msgs::Action>::iterator it2 = itms->todoActions.begin(); it2 != itms->todoActions.end(); it2++){
                            if(areActionsEqual(*it, *it2)){
                                find = true;
                                break;
                            }
                        }
                        if(!find){
                            //we look for the reason and try to solve it
                            solveDB(itms->agentName, *itms, *it, true);

                        }
                    }
                }
                //we check if the actions todo of the agent are really todo
                for(std::vector<supervisor_msgs::Action>::iterator it2 = itms->todoActions.begin(); it2 != itms->todoActions.end(); it2++){
                    //we check only human actions
                    if(isInList(itms->agentName, it2->actors)){
                        bool find = false;
                        for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo_.begin(); it != actionsTodo_.end(); it++){
                            if(areActionsEqual(*it, *it2)){
                                find = true;
                                break;
                            }
                        }
                        if(!find){
                            //we look for the reason and try to solve it
                            solveDB(itms->agentName, *itms, *it2, false);
                        }
                    }
                }
                //if no action todo for the human, we check if the actions todo for the x agent are the same
                if(!actionTodo){
                    for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo_.begin(); it != actionsTodo_.end(); it++){
                        //we check only agent X actions
                        if(isInList(xAgent_, it->actors)){
                            actionTodo = true;
                            bool find = false;
                            for(std::vector<supervisor_msgs::Action>::iterator it2 = itms->todoActions.begin(); it2 != itms->todoActions.end(); it2++){
                                if(areActionsEqual(*it, *it2)){
                                    find = true;
                                    break;
                                }
                            }
                            if(!find){
                                //we look for the reason and try to solve it
                                solveDB(itms->agentName, *itms, *it, true);
                            }
                        }
                    }
                    //we check if the actions todo of the agent are really todo
                    for(std::vector<supervisor_msgs::Action>::iterator it2 = itms->todoActions.begin(); it2 != itms->todoActions.end(); it2++){
                        //we check only agent X actions
                        if(isInList(xAgent_, it2->actors)){
                            bool find = false;
                            for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo_.begin(); it != actionsTodo_.end(); it++){
                                if(it->id == it2->id){
                                    find = true;
                                    break;
                                }
                            }
                            if(!find){
                                //we look for the reason and try to solve it
                                solveDB(itms->agentName, *itms, *it2, false);
                            }
                        }
                    }
                }
            }
        }
    }
}


/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "robot_decision");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[robot_decision] Init");

  node.getParam("supervisor/robot/name", robotName_);
  node.getParam("supervisor/AgentX", xAgent_);
  node.getParam("robot_decision/mode", mode_);
  node_->getParam("/supervisor/mainPartner", mainPartner_);
  node_->getParam("/robot_decision/timeAdaptation", timeAdaptation_);
  node_->getParam("/robot_decision/timeWaitHuman", timeWaitHuman_);
  node_->getParam("/robot_decision/areaInform", areaInform_);
  node_->getParam("/robot_decision/areaIdle", areaIdle_);
  node_->getParam("/action_executor/restPosition/right", rightRestPosition_);
  node_->getParam("/action_executor/restPosition/left", leftRestPosition_);

  robotState_ = "IDLE";
  timerStarted_ = false;
  shouldRetractRight_ = true;
  shouldRetractLeft_ = true;
  isGivingInfo_ = false;
  hasActed_ = false;
  planPrevId_ = -1;
  msPrevId_ = -1;
  prevRobotId_ = -1;

  fillHighLevelNames();

  ros::ServiceClient client_ask = node_->serviceClient<supervisor_msgs::Ask>("dialogue_node/ask");
  client_ask_ = &client_ask;
  ros::ServiceClient client_end_plan = node_->serviceClient<supervisor_msgs::EndPlan>("plan_elaboration/end_plan");
  client_end_plan_ = &client_end_plan;
  ros::ServiceClient client_inform = node_->serviceClient<supervisor_msgs::GiveInfo>("dialogue_node/give_info");
  client_inform_ = &client_inform;
  ros::ServiceClient client_execute_db = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
  client_execute_db_ = &client_execute_db;
  ros::ServiceClient client_get_info_db = node_->serviceClient<toaster_msgs::GetInfoDB>("database_manager/get_info");
  client_get_info_db_ = &client_get_info_db;


  ROS_INFO("[robot_decision] Waiting for action executor server");
  Client actionClient("supervisor/action_executor", true);
  actionClient.waitForServer();
  actionClient_ = &actionClient;

  ros::Subscriber sub_todo = node.subscribe("supervisor/actions_todo", 1,todoCallback);
  ros::Subscriber sub_ms = node.subscribe("mental_states/mental_states", 1, msCallback);
  ros::Subscriber sub_goal = node.subscribe("goal_manager/goalsList", 1, goalCallback);
  ros::Subscriber sub_area = node.subscribe("area_manager/factList", 1, areaFactListCallback);
  ros::Subscriber sub_plan = node.subscribe("plan_elaboration/plan", 1, planCallback);
  ros::Subscriber sub_prev = node.subscribe("mental_states/previous_actions", 1, previousCallback);


  ros::ServiceServer service_stop = node.advertiseService("robot_decision/stop", stopSrv); //when an action needs to be stopped

  ROS_INFO("[robot_decision] Ready");

  while(node.ok()){
      //activate the readers
      ros::spinOnce();
      loop_rate.sleep();
  }
}

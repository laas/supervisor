/**
author Sandra Devin

State machine for the robot
**/

#include <state_machines/robot_sm.h>



RobotSM::RobotSM(ros::NodeHandle* node):
actionClient_("supervisor/action_executor", true)
 {
    node_ = node;
    node_->getParam("/robot/name", robotName_);
    node_->getParam("/HATP/AgentX", agentX_);
	actionClient_.waitForServer();
	isActing_ = false;
    shouldRetractRight_ = true;
    shouldRetractLeft_ = true;
    fillHighLevelNames();
    node_->getParam("/negociationMode", negociationMode_);
}


/*
Fill the highLevelNames map from param
*/
void RobotSM::fillHighLevelNames(){

    //we retrieve the list objects from param and fill the map with their high level name
    vector<string> objects;
    node_->getParam("/entities/objects", objects);
    for(vector<string>::iterator it = objects.begin(); it != objects.end(); it++){
        string topic = "/highLevelName/";
        topic = topic + *it;
        string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for supports
    vector<string> support;
    node_->getParam("/entities/supports", support);
    for(vector<string>::iterator it = support.begin(); it != support.end(); it++){
        string topic = "/highLevelName/";
        topic = topic + *it;
        string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for containers
    vector<string> container;
    node_->getParam("/entities/containers", container);
    for(vector<string>::iterator it = container.begin(); it != container.end(); it++){
        string topic = "/highLevelName/";
        topic = topic + *it;
        string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
}


/*
Called once when the goal of the action client completes
*/
void RobotSM::doneCb(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result){

		isActing_ = false;
        shouldRetractRight_ = result->shouldRetractRight;
        shouldRetractLeft_ = result->shouldRetractLeft;
}

/*
State where the robot is IDLE
*/
string RobotSM::idleState(vector<string> partners, map<string, string> agentsState){

    partners_ = partners;

    vector<supervisor_msgs::ActionMS> actionsR = getActionReady(robotName_, robotName_);
    if(actionsR.size()){
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action = convertActionMStoAction(actionsR[0]);
        actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
        isActing_ = true;
        ROS_INFO("[state_machines] Robot goes to ACTING");
        return "ACTING";
    }else{
        vector<supervisor_msgs::ActionMS> actionsX = getActionReady(agentX_, robotName_);
        if(actionsX.size()){
            vector<supervisor_msgs::ActionMS> identicals = getIdenticalActions(actionsX);
            bool identical;
            supervisor_msgs::ActionMS actionChosen;
            if(identicals.size()){
                identical = true;
                actionChosen = identicals[0];
            }else{
                identical = false;
                actionChosen = actionsX[0];
            }
            vector<string> actorsR = getPossibleActors(actionChosen, robotName_, agentsState);
            for(vector<string>::iterator it = partners_.begin(); it != partners_.end(); it++){
                vector<string> actorsH = getPossibleActors(actionChosen, *it, agentsState);
                if(actorsR.size() != actorsH.size()){
                    //CorrectDB
                    return "IDLE";
                }
            }
            if(actorsR.size()){
                if(actorsR.size() == 1){
                    attributeAction(actionChosen, actorsR[0]);
                }else{
                    if(identical){
                        //we attribute to the robot
                        attributeAction(actionChosen, robotName_);
                    }else{
                        //we negotiate or adapt to choose an actor
                        if(negociationMode_){
                            ros::ServiceClient client_ask = node_->serviceClient<supervisor_msgs::Ask>("dialogue_node/ask");
                            supervisor_msgs::Ask srv_ask;
                            srv_ask.request.type = "ACTION";
                            srv_ask.request.subType = "WANT";
                            srv_ask.request.action = convertActionMStoAction(actionChosen);
                            srv_ask.request.receiver = partners_[0];
                            srv_ask.request.waitForAnswer = true;
                            if (client_ask.call(srv_ask)){
                              if(srv_ask.response.boolAnswer){
                                  //the partner want to perform the action
                                  attributeAction(actionChosen, partners_[0]);
                              }else{
                                  //the partner does not want to perform the action: the robot does it
                                  attributeAction(actionChosen, robotName_);
                              }
                            }else{
                              ROS_ERROR("Failed to call service dialogue_node/ask");
                            }
                        }else{
                            //TODO: we wait few time to see if the partner performs the action, if not the robot does it

                        }
                    }
                }
            }else{
                //no possible actor: what to do?
            }
        }else{
            //NEXT ROBOT ACTION?
        }
    }

    //if no more action to do we retract if needed
    if(shouldRetractRight_){
        //retract right arm if needed
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action.name = "moveTo";
        string rightArmRestPose_;
        node_->getParam("/restPosition/right", rightArmRestPose_);
        goal.action.parameters.push_back(rightArmRestPose_);
        goal.action.actors.push_back(robotName_);
        actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
        isActing_ = true;
        ROS_INFO("[state_machines] Robot goes to ACTING");
        return "ACTING";
    }
    if(shouldRetractLeft_){
        //retract left arm if needed
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action.name = "moveTo";
        string leftArmRestPose_;
        node_->getParam("/restPosition/left", leftArmRestPose_);
        goal.action.parameters.push_back(leftArmRestPose_);
        goal.action.actors.push_back(robotName_);
        actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
        isActing_ = true;
        ROS_INFO("[state_machines] Robot goes to ACTING");
        return "ACTING";
    }
	return "IDLE";
}

/*
State where the robot is ACTING
*/
string RobotSM::actingState(){

    //get objects and weight from action_executor topic


	if(!isActing_){
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
	}

	return "ACTING";
}

/*
State where the robot is WAITING
*/
string RobotSM::waitingState(){

	//We look if the robot has an action to do
    ros::ServiceClient client = node_->serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    supervisor_msgs::GetInfo srv;
    srv.request.info ="ACTIONS_TODO";
	srv.request.agent = robotName_;
	srv.request.actor = robotName_;
	if (client.call(srv)){
	 if(srv.response.state == "READY"){//the robot has an action to do, we send it to the action manager
		supervisor_msgs::ActionExecutorGoal goal;
  		goal.action = srv.response.action;
  		actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
		isActing_ = true;
		ROS_INFO("[state_machines] Robot goes to ACTING");
		return "ACTING";
	 }else if(srv.response.state == "NEEDED"){//the robot is still not possible, we stay in the WAITING state
		return "WAITING";
	 }else{// the robot has no more action to do, we return to IDLE
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
	  }
	}else{
     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
	}
	
	return "WAITING";
}

/*
Get the actions ready for an agent in another agent knowledge
    - @actor: the agent we look the actions
    - @agent: the agent in which knowledge we look
 */
vector<supervisor_msgs::ActionMS> RobotSM::getActionReady(string actor, string agent){

    vector<supervisor_msgs::ActionMS> answer;
    for(vector<supervisor_msgs::AgentKnowledge>::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->agentName == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                if(itk->property == "actionState" && (itk->targetId == "READY" || itk->targetId == "ASKED")){
                    //we got a ready action, now we need to check the actors
                    istringstream buffer(itk->subjectId);
                    int id;
                    buffer >> id;
                    supervisor_msgs::ActionMS action = getActionFromId(id);
                    for(vector<string>::iterator ita = action.actors.begin(); ita != action.actors.end(); ita++){
                        if(*ita == actor){
                            //if it is an asked action, we chek if the action is feasible
                            if(itk->targetId == "ASKED"){
                                if(factsAreIn(agent, action.prec)){
                                    answer.push_back(action);
                                }
                            }
                            else{
                                answer.push_back(action);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    return answer;

}

/*
Return an action from an id
 */
supervisor_msgs::ActionMS RobotSM::getActionFromId(int id){

    for(vector<supervisor_msgs::ActionMS>::iterator it = actions_.begin(); it != actions_.end(); it++){
        if(it->id == id){
            return *it;
        }
    }

    supervisor_msgs::ActionMS answer;
    return answer;
}

/*
Convert an action in the ActionMS format to Action format
 */
supervisor_msgs::Action RobotSM::convertActionMStoAction(supervisor_msgs::ActionMS actionMS){

    supervisor_msgs::Action action;
    action.name = actionMS.name;
    action.id = actionMS.id;
    action.parameters = actionMS.parameters;
    action.actors = actionMS.actors;

    return action;
}

/*
Function which return true if all the facts given are in the agent knowledge
    @agent: the agent name
    @facts: the facts we are looking for
*/
bool RobotSM::factsAreIn(string agent, vector<toaster_msgs::Fact> facts){

    for(vector<supervisor_msgs::AgentKnowledge>::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->agentName == agent){
            for(vector<toaster_msgs::Fact>::iterator itf = facts.begin(); itf != facts.end(); itf++){
                if(itf->subjectId == "NULL"){
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                        if(itk->property == itf->property && highLevelNames_[itk->targetId] == itf->targetId){
                            return false;
                        }
                    }
                }else if(itf->targetId == "NULL"){
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                        if(itk->property == itf->property && highLevelNames_[itk->subjectId] == itf->subjectId){
                            return false;
                        }
                    }
                }else{
                    bool find = false;
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                        if(itk->property == itf->property &&
                                highLevelNames_[itk->subjectId] == itf->subjectId &&
                                highLevelNames_[itk->targetId] == itf->targetId){
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
Function which return the actions in a list which have another identical action in the same list
*/
vector<supervisor_msgs::ActionMS> RobotSM::getIdenticalActions(vector<supervisor_msgs::ActionMS> actions){

    vector<supervisor_msgs::ActionMS> answer;
    for(vector<supervisor_msgs::ActionMS>::iterator it = actions.begin(); it != actions.end(); it++){
        for(vector<supervisor_msgs::ActionMS>::iterator it2 = it+1; it2 != actions.end(); it2++){
            if(areIdentical(*it, *it2)){
                answer.push_back(*it);
                actions.erase(it2);
                break;
            }
        }
    }

    return answer;
}

/*
Function which return true if two actions are sementically the same (same name, actors and parameters)
*/
bool RobotSM::areIdentical(supervisor_msgs::ActionMS action1, supervisor_msgs::ActionMS action2){

    if(action1.name != action2.name){
        return false;
    }else{
        if(action1.actors.size() != action2.actors.size()){
            return false;
        }else{
            vector<string>::iterator it2 = action2.actors.begin();
            for(vector<string>::iterator it1 = action1.actors.begin(); it1 != action1.actors.end(); it1++){
                if(*it1 != *it2){
                    return false;
                }
                it2++;
            }
            if(action1.parameters.size() != action2.parameters.size()){
                return false;
            }else{
                vector<string>::iterator it2 = action2.parameters.begin();
                for(vector<string>::iterator it1 = action1.parameters.begin(); it1 != action1.parameters.end(); it1++){
                    if(*it1 != *it2){
                        return false;
                    }
                    it2++;
                }
            }
        }
    }

    return true;
}


/*
Function which return the possible actors for an action (actors who check the preconditions) taking the point of view of @agent
*/
vector<string> RobotSM::getPossibleActors(supervisor_msgs::ActionMS action, string agent, map<string, string> agentsState){

    vector<string> answer;
    vector<string> goalActors = partners_;
    goalActors.push_back(robotName_);

    for(vector<string>::iterator ita = goalActors.begin(); ita != goalActors.end(); ita++){
        if(agent == robotName_ || agentsState[agent] != "ACTING"){//agents already performing an action are not possible actors
            vector<toaster_msgs::Fact> newPrecs;
            for(vector<toaster_msgs::Fact>::iterator itp = action.prec.begin(); itp != action.prec.end(); itp++){
                toaster_msgs::Fact fact = *itp;
                if(fact.subjectId == agentX_){
                    fact.subjectId = *ita;
                }
                if(fact.targetId == agentX_){
                    fact.targetId = *ita;
                }
                newPrecs.push_back(fact);
            }
            if(factsAreIn(agent, newPrecs)){
                answer.push_back(*ita);
            }
        }
    }

    return answer;
}


/*
Function which attribute an action to an agent
*/
void RobotSM::attributeAction(supervisor_msgs::ActionMS action, string agent){

    //find the name of the high level object to lock
    string highLevelObject = getHighLevelLockedObject(action);

    //find the object corresponding to the high level object
    string objectLocked = getCorrespondingObject(action, highLevelObject, agent);

    //lock the object and ask for a new plan
    ros::ServiceClient client = node_->serviceClient<supervisor_msgs::EndPlan>("plan_elaboration/endPlan");
    supervisor_msgs::EndPlan srv;
    srv.request.report = false;
    srv.request.objectLocked = objectLocked;
    srv.request.agentLocked = agent;
    if (!client.call(srv)){
      ROS_ERROR("Failed to call service plan_elaboration/endPlan");
    }
}


/*
Function which return te object to lock given an action
TODO: find a better way to do it
*/
string RobotSM::getHighLevelLockedObject(supervisor_msgs::ActionMS action){

    //For now, for each possible action the first parameters is always the object to lock (object to  pick, ...)
    return action.parameters[0];
}

/*
Function which instantiate the object to lock
*/
string RobotSM::getCorrespondingObject(supervisor_msgs::ActionMS action, string highLevelObject, string agent){

    string object = "NOT_FOUND";

    for(vector<toaster_msgs::Fact>::iterator itp = action.prec.begin(); itp != action.prec.end(); itp++){
        if(itp->subjectId != highLevelObject && itp->targetId != highLevelObject){
            //this precondition does not concern the object
            break;
        }else if(itp->subjectId == "NULL" || itp->targetId == "NULL"){
            break;
        }else{
            toaster_msgs::Fact prec = *itp;
            if(prec.subjectId == agentX_){
                prec.subjectId = agent;
            }
            if(prec.targetId == agentX_){
                prec.targetId = agent;
            }
            bool isSubject;
            if(itp->subjectId == highLevelObject){
                isSubject = true;
            }else{
                isSubject = false;
            }
            for(vector<supervisor_msgs::AgentKnowledge>::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
                if(it->agentName == agent){
                    for(vector<toaster_msgs::Fact>::iterator itf = it->knowledge.begin(); itf != it->knowledge.end(); itf++){
                        if(itf->property == prec.property &&
                                highLevelNames_[itf->subjectId] == highLevelNames_[prec.subjectId] &&
                                highLevelNames_[itf->targetId] == highLevelNames_[prec.targetId]){
                            //we get an object checking this precondition for the agent
                            if(isSubject){
                                object = itf->subjectId;
                            }else{
                                object = itf->targetId;
                            }
                            break;
                            //TODO check other preconditions with this object
                        }
                    }
                }
            }

        }
    }

    return object;
}

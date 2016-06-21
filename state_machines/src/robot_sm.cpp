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
string RobotSM::idleState(vector<string> partners){

    /*//We look if the robot has an action to do
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
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
	 }else if(srv.response.state == "NEEDED"){//the robot has an action but not possible yet, we go to the WAITING state
		ROS_INFO("[state_machines] Robot goes to WAITING");
		return "WAITING";
	 }
	}else{
     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
	}
    */
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
            vector<string> actorsR = getPossibleActors(actionChosen, robotName_);
            for(vector<string>::iterator it = partners_.begin(); it != partners_.end(); it++){
                vector<string> actorsH = getPossibleActors(actionChosen, *it);
                if(actorsR.size() != actorsH.size()){
                    //CorrectDB
                    return "IDLE";
                }
            }
            if(actorsR.size()){
                if(actorsR.size() == 1){
                    //we attribute to the only possible actor
                }else{
                    if(identical){
                        //we attribute to the robot
                    }else{
                        //we negotiate or adapt to choose an actor
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
                        string topic = "/highLevelName/";
                        topic = topic + itk->targetId;
                        string higlLevelTargetName;
                        if(node_->hasParam(topic)){
                            node_->getParam(topic, higlLevelTargetName);
                        }else{
                            higlLevelTargetName = itk->targetId;
                        }
                        if(itk->property == itf->property && higlLevelTargetName == itf->targetId){
                            return false;
                        }
                    }
                }else if(itf->targetId == "NULL"){
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                        string topic = "/highLevelName/";
                        topic = topic + itk->subjectId;
                        string higlLevelSubjectName;
                        if(node_->hasParam(topic)){
                            node_->getParam(topic, higlLevelSubjectName);
                        }else{
                            higlLevelSubjectName = itk->subjectId;
                        }
                        if(itk->property == itf->property && higlLevelSubjectName == itf->subjectId){
                            return false;
                        }
                    }
                }else{
                    bool find = false;
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                        string topic = "/highLevelName/";
                        topic = topic + itk->subjectId;
                        string higlLevelSubjectName;
                        if(node_->hasParam(topic)){
                            node_->getParam(topic, higlLevelSubjectName);
                        }else{
                            higlLevelSubjectName = itk->subjectId;
                        }
                        topic = "/highLevelName/";
                        topic = topic + itk->targetId;
                        string higlLevelTargetName;
                        if(node_->hasParam(topic)){
                            node_->getParam(topic, higlLevelTargetName);
                        }else{
                            higlLevelTargetName = itk->targetId;
                        }
                        if(itk->property == itf->property && higlLevelSubjectName == itf->subjectId && higlLevelTargetName == itf->targetId){
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
vector<string> RobotSM::getPossibleActors(supervisor_msgs::ActionMS action, string agent){

    vector<string> answer;

    for(vector<string>::iterator ita = partners_.begin(); ita != partners_.end(); ita++){
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

    return answer;
}

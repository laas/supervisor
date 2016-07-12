/**
author Sandra Devin

State machine for the robot
**/

#include <state_machines/robot_sm.h>



RobotSM::RobotSM():
actionClient_("supervisor/action_executor", true)
 {
    node_.getParam("/robot/name", robotName_);
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
string RobotSM::idleState(){

    vector<supervisor_msgs::ActionMS> actionsReady = getActionReady(robotName_, robotName_);
    if(actionsReady.size()){
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action = convertActionMStoAction(actionsReady[0]);
        actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
        isActing_ = true;
        ROS_INFO("[state_machines] Robot goes to ACTING");
        return "ACTING";
    }else{
        vector<supervisor_msgs::ActionMS> actionsNeeded = getActionNeeded(robotName_, robotName_);
        if(actionsNeeded.size()){
            ROS_INFO("[state_machines] Robot goes to WAITING");
            return "WAITING";
        }
    }

    //if no more action to do we retract if needed
    if(shouldRetractRight_){
        //retract right arm if needed
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action.name = "moveTo";
        string rightArmRestPose_;
        node_.getParam("/restPosition/right", rightArmRestPose_);
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
        node_.getParam("/restPosition/left", leftArmRestPose_);
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

    vector<supervisor_msgs::ActionMS> actionsReady = getActionReady(robotName_, robotName_);
    if (actionsReady.size()){//the robot has an action to do, we send it to the action manager
		supervisor_msgs::ActionExecutorGoal goal;
        goal.action = convertActionMStoAction(actionsReady[0]);
  		actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
		isActing_ = true;
		ROS_INFO("[state_machines] Robot goes to ACTING");
		return "ACTING";
     }else{
        vector<supervisor_msgs::ActionMS> actionsNeeded = getActionNeeded(robotName_, robotName_);
        if(actionsNeeded.size()){//the robot is still not possible, we stay in the WAITING state
            return "WAITING";
        }else{// the robot has no more action to do, we return to IDLE
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
        }
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
Get the actions needed for an agent in another agent knowledge
    - @actor: the agent we look the actions
    - @agent: the agent in which knowledge we look
 */
vector<supervisor_msgs::ActionMS> RobotSM::getActionNeeded(string actor, string agent){

    vector<supervisor_msgs::ActionMS> answer;
    for(vector<supervisor_msgs::AgentKnowledge>::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->agentName == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                if(itk->property == "actionState" && (itk->targetId == "NEEDED" || itk->targetId == "ASKED")){
                    //we got a ready action, now we need to check the actors
                    istringstream buffer(itk->subjectId);
                    int id;
                    buffer >> id;
                    supervisor_msgs::ActionMS action = getActionFromId(id);
                    for(vector<string>::iterator ita = action.actors.begin(); ita != action.actors.end(); ita++){
                        if(*ita == actor){
                            //if it is an asked action, we chek if the action is not feasible
                            if(itk->targetId == "ASKED"){
                                if(!factsAreIn(agent, action.prec)){
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
                        if(itk->property == itf->property && itk->targetId == itf->targetId){
                            return false;
                        }
                    }
                }else if(itf->targetId == "NULL"){
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                        if(itk->property == itf->property && itk->subjectId == itf->subjectId){
                            return false;
                        }
                    }
                }else{
                    bool find = false;
                    for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
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

/**
author Sandra Devin

State machine for the robot
**/

#include <state_machines/human_sm.h>



HumanSM::HumanSM(string humanName)
 {
	humanName_ = humanName;
    node_.getParam("/robot/name", robotName_);
	node_.getParam("/timeNoAction", timeToWait_);
    node_.getParam("/timeSignalingHuman", timeSignaling_);
  	node_.getParam("/simu", simu_);
	timerStarted_ = false;
    present_ = true;
    signalGiven_ = false;

    subArea_ = node_.subscribe("area_manager/factList", 1000, &HumanSM::areaFactListCallback, this);
}

void HumanSM::areaFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){

    string area;
    node_.getParam("/areaPresence", area);
    bool found = false;
    vector<toaster_msgs::Fact> areaFacts = msg->factList;
    for(vector<toaster_msgs::Fact>::iterator it = areaFacts.begin(); it != areaFacts.end(); it++){
        if(it->subjectId == humanName_ && it->property == "IsInArea" && it->targetId == area){
            found = true;
            break;
        }
    }
    present_ = found;

}


/*
 Determine the focus of an action
 */
string HumanSM::focusObject(supervisor_msgs::Action action){

    string object;

    if(action.name == "pick"){
        if(action.parameters.size() > 0){
            object = action.parameters[0];
        }
    }else if(action.name == "place"){
        if(action.parameters.size() > 1){
            object = action.parameters[1];
        }
    }else if(action.name == "pickandplace"){
        if(action.parameters.size() > 1){
            object = action.parameters[1];
        }
    }else if(action.name == "drop"){
        if(action.parameters.size() > 1){
            object = action.parameters[1];
        }
    }else if(action.name == "pickanddrop"){
        if(action.parameters.size() > 1){
            object = action.parameters[1];
        }
    }

    return object;
}
/*
 Determine the objects and duration to look when we want to give a signal for an action to do
 */
pair<vector<string>, vector<double> > HumanSM::signalObjects(supervisor_msgs::Action action){

    vector<string> objects;
    vector<double> durations;

    if(action.name == "pick"){
        if(action.parameters.size() > 0){
            objects.push_back(action.parameters[0]);
            durations.push_back(0.0);
        }
    }else if(action.name == "place"){
        if(action.parameters.size() > 1){
            objects.push_back(action.parameters[0]);
            durations.push_back(0.0);
            objects.push_back(action.parameters[1]);
            durations.push_back(0.0);
        }
    }else if(action.name == "pickandplace"){
        if(action.parameters.size() > 1){
            objects.push_back(action.parameters[0]);
            durations.push_back(0.0);
            objects.push_back(action.parameters[1]);
            durations.push_back(0.0);
        }
    }else if(action.name == "drop"){
        if(action.parameters.size() > 1){
            objects.push_back(action.parameters[0]);
            durations.push_back(0.0);
            objects.push_back(action.parameters[1]);
            durations.push_back(0.0);
        }
    }else if(action.name == "pickanddrop"){
        if(action.parameters.size() > 1){
            objects.push_back(action.parameters[0]);
            durations.push_back(0.0);
            objects.push_back(action.parameters[1]);
            durations.push_back(0.0);
        }
    }
    pair<vector<string>, vector<double> > toReturn;
    toReturn.first = objects;
    toReturn.second = durations;

    return toReturn;
}

/*
State where the human is IDLE
*/
string HumanSM::idleState(){

    if(!present_){
        ROS_INFO("[state_machines] %s goes to ABSENT", humanName_.c_str());
        return "ABSENT";
    }
    if(humanActs_){
        ROS_INFO("[state_machines] %s goes to ACTING", humanName_.c_str());
        return "ACTING";
    }

    //We look if the human thinks he has an action to do
    ros::ServiceClient client_db = node_.serviceClient<supervisor_msgs::SolveDivergentBelief>("mental_state/solve_divergent_belief");
    supervisor_msgs::SolveDivergentBelief srv_db;
    vector<supervisor_msgs::ActionMS> actionsReady = getActionReady(humanName_, humanName_);
     if(actionsReady.size()){//the human thinks he has an action to do
         string robotState = getActionState(actionsReady[0], robotName_);
         if(robotState== "READY"){//the state is the same in the robot knowledge, the human SOULD ACT
            shouldDoAction_ = convertActionMStoAction(actionsReady[0]);
			ROS_INFO("[state_machines] %s goes to SHOULD ACT", humanName_.c_str());
			return "SHOULDACT";
		 }else{//it is necessary to solve the divergent belief
            srv_db.request.action = convertActionMStoAction(actionsReady[0]);
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
		  }
        }else{
         vector<supervisor_msgs::ActionMS> actionsNeeded = getActionNeeded(humanName_, humanName_);
         if(actionsNeeded.size()){//the human thinks he has an action to do but no possible
         string robotState = getActionState(actionsNeeded[0], robotName_);
         if(robotState == "NEEDED"){//the state is the same in the robot knowledge, the human has to WAIT
			ROS_INFO("[state_machines] %s goes to WAITING", humanName_.c_str());
			return "WAITING";
         }else if(robotState == "READY"){//the robot thinks the human can act, it is necessary to solve the divergent belief
            srv_db.request.action = convertActionMStoAction(actionsNeeded[0]);
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
          }
         }else{
             vector<supervisor_msgs::ActionMS> actionsRobotReady = getActionReady(humanName_, robotName_);
             if(actionsRobotReady.size()){
                 srv_db.request.action = convertActionMStoAction(actionsRobotReady[0]);
                 if (!client_db.call(srv_db)){
                     ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
                 }
             }
         }
     }

	return "IDLE";
}

/*
State where the human is ACTING
*/
string HumanSM::actingState(string* object, bool* unexpected){

    //we compare the action to perform and the action performed in order to know if the action is unexpected
    *unexpected = true;
    if(shouldDoAction_.name != PerformedAction_.name){
        *unexpected = true;
    }else{
        if(shouldDoAction_.parameters.size() != PerformedAction_.parameters.size()){
            *unexpected = true;
        }else{
            for(int i = 0; i < shouldDoAction_.parameters.size(); i++){
                if(shouldDoAction_.parameters[i] != PerformedAction_.parameters[i]){
                    *unexpected = true;
                    break;
                }
            }
        }
    }

    //we determine the objects of focus
    *object = focusObject(PerformedAction_);

    shouldDoAction_.name = "NULL";
    PerformedAction_.name = "NULL";
    humanActs_ = false;

    return "IDLE";
}

/*
State where the human is ABSENT
*/
string HumanSM::absentState(){

    if(present_){
        ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
        return "IDLE";
    }

	return "ABSENT";
}

/*
State where the human is WAITING
*/
string HumanSM::waitingState(){

    if(!present_){
        ROS_INFO("[state_machines] %s goes to ABSENT", humanName_.c_str());
        return "ABSENT";
    }
    if(humanActs_){
        ROS_INFO("[state_machines] %s goes to ACTING", humanName_.c_str());
        return "ACTING";
    }
	
    //We look if the human still thinks he has an action to do
    ros::ServiceClient client_db = node_.serviceClient<supervisor_msgs::SolveDivergentBelief>("mental_state/solve_divergent_belief");
    supervisor_msgs::SolveDivergentBelief srv_db;
    vector<supervisor_msgs::ActionMS> actionsNeeded = getActionNeeded(humanName_, humanName_);
     if(actionsNeeded.size()){
        //we verify that the robot also thinks the action is NEEDED
        string robotState = getActionState(actionsNeeded[0], robotName_);
         if(robotState == "NEEDED"){
			//the state is the same in the robot knowledge, the human continue to WAIT
			return "WAITING";
		 }else{//there is a divergent belief, we solve it and return to IDLE
            srv_db.request.action = convertActionMStoAction(actionsNeeded[0]);
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
			ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
			return "IDLE";
          }
	 }else{//we return to IDLE to look for other actions
		ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
		return "IDLE";
      }
	
	return "WAITING";
}

/*
State where the human SHOULD ACT
*/
string HumanSM::shouldActState(string robotState){

    if(!present_){
        ROS_INFO("[state_machines] %s goes to ABSENT", humanName_.c_str());
        return "ABSENT";
    }
    if(humanActs_){
        ROS_INFO("[state_machines] %s goes to ACTING", humanName_.c_str());
        return "ACTING";
    }

    ros::ServiceClient action_state = node_.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
    supervisor_msgs::ChangeState srv_action;

    if(!timerStarted_ && robotState =="WAITING"){//we just enter the state, we start the timer
		start_ = clock();
		timerStarted_ = true;
        signalGiven_ = false;
    }else if(timerStarted_ && robotState !="WAITING"){
        timerStarted_ = false;
        signalGiven_ = false;
    }else{
        vector<supervisor_msgs::ActionMS> actionsReady = getActionNeeded(humanName_, humanName_);
            if(actionsReady.size()){
                shouldDoAction_ = convertActionMStoAction(actionsReady[0]);
                pair<vector<string>, vector<double> > objects = signalObjects(convertActionMStoAction(actionsReady[0]));
                double duration = (clock() - start_ ) / (double) CLOCKS_PER_SEC;
                if(duration >= timeSignaling_){//we send a head signal concerning the action
                    ros::Publisher signal_pub = node_.advertise<head_manager::Signal>("head_manager/signal", 1000);
                    head_manager::Signal msg;

                    msg.receivers.push_back(humanName_);
                    msg.entities = objects.first;
                    msg.durations = objects.second;
                    msg.urgency = 0.5;
                    msg.importancy = 0.6;
                    msg.weight = 0.6;
                    signal_pub.publish(msg);
                    signalGiven_ = true;
                }
                if(duration >= timeToWait_){
                    timerStarted_ = false;
                    signalGiven_ = false;
                    string robotState = getActionState(actionsReady[0], robotName_);
                     if(robotState != "ASKED"){//if the action is not already ASKED, the robot asks to do the action
                        ros::ServiceClient client_ask = node_.serviceClient<supervisor_msgs::Ask>("dialogue_node/ask");
                        supervisor_msgs::Ask srv_ask;
                        srv_ask.request.type = "ACTION";
                        srv_ask.request.subType = "CAN";
                        srv_ask.request.action = convertActionMStoAction(actionsReady[0]);
                        srv_ask.request.receiver = humanName_;
                      }else{//else, we consider the action failed
                         srv_action.request.type = "action";
                         srv_action.request.action = convertActionMStoAction(actionsReady[0]);
                         srv_action.request.state = "FAILED";
                          if (!action_state.call(srv_action)){
                            ROS_ERROR("Failed to call service mental_state/change_state");
                          }
                       ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
                       shouldDoAction_.name = "NULL";
                       return "IDLE";
                      }
            }else{//there is no more action to do
                ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
                shouldDoAction_.name = "NULL";
                return "IDLE";
            }
		}
	}
	
    shouldDoAction_.name = "NULL";
	return "SHOULDACT";
}



/*
Get the actions ready for an agent in another agent knowledge
    - @actor: the agent we look the actions
    - @agent: the agent in which knowledge we look
 */
vector<supervisor_msgs::ActionMS> HumanSM::getActionReady(string actor, string agent){

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
vector<supervisor_msgs::ActionMS> HumanSM::getActionNeeded(string actor, string agent){

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
Get the state of an action from an agent point of view
 */
string HumanSM::getActionState(supervisor_msgs::ActionMS action, string agent){

    string answer = "NOT_FOUND";
    for(vector<supervisor_msgs::AgentKnowledge>::iterator it = knowledge_.begin(); it != knowledge_.end(); it++){
        if(it->agentName == agent){
            for(vector<toaster_msgs::Fact>::iterator itk = it->knowledge.begin(); itk != it->knowledge.end(); itk++){
                istringstream buffer(itk->subjectId);
                int id;
                buffer >> id;
                if(itk->property == "actionState" && id == action.id){
                    return itk->targetId;
                }
            }
        }
    }

    return answer;

}

/*
Return an action from an id
 */
supervisor_msgs::ActionMS HumanSM::getActionFromId(int id){

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
supervisor_msgs::Action HumanSM::convertActionMStoAction(supervisor_msgs::ActionMS actionMS){

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
bool HumanSM::factsAreIn(string agent, vector<toaster_msgs::Fact> facts){

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

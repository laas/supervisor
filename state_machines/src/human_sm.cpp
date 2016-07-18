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
     ROS_INFO("[state_machines] Waiting for head action server");
    head_action_client = new actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>("pr2motion/Head_Move_Target",true);
    head_action_client->waitForServer();
    ROS_INFO("[state_machines] Human state machine ready");
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
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
  	ros::ServiceClient client_db = node_.serviceClient<supervisor_msgs::SolveDivergentBelief>("mental_state/solve_divergent_belief");
    supervisor_msgs::GetInfo srv_info;
    supervisor_msgs::SolveDivergentBelief srv_db;
    srv_info.request.info ="ACTIONS_TODO";
    srv_info.request.agent = humanName_;
    srv_info.request.actor = humanName_;
	srv_db.request.agent = humanName_;
    if (client.call(srv_info)){
     if(srv_info.response.state == "READY"){//the human thinks he has an action to do
        //we look if the robot also thinks the human should do the action
        srv_info.request.info ="ACTION_STATE";
        srv_info.request.agent = robotName_;
        srv_info.request.action = srv_info.response.action;
        if (client.call(srv_info)){
         if(srv_info.response.state == "READY"){//the state is the same in the robot knowledge, the human SOULD ACT
            shouldDoAction_ = srv_info.response.action;
			ROS_INFO("[state_machines] %s goes to SHOULD ACT", humanName_.c_str());
			return "SHOULDACT";
		 }else{//it is necessary to solve the divergent belief
            srv_db.request.action = srv_info.response.action;
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
		  }
		}else{
	 	 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_state");
		}
     }else if(srv_info.response.state == "NEEDED"){//the human thinks he has an action to do but no possible
        //we look if the robot also thinks the human should do the action and that the action is not possible
        srv_info.request.info ="ACTION_STATE";
        srv_info.request.agent = robotName_;
        srv_info.request.action = srv_info.response.action;
        if (client.call(srv_info)){
         if(srv_info.response.state == "NEEDED"){//the state is the same in the robot knowledge, the human has to WAIT
			ROS_INFO("[state_machines] %s goes to WAITING", humanName_.c_str());
			return "WAITING";
         }else if(srv_info.response.state == "READY"){//the robot thinks the human can act, it is necessary to solve the divergent belief
            srv_db.request.action = srv_info.response.action;
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
		  }
		}else{
         ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
		}
	 }else{//the human thinks he has no action, we check if the robot thinks he has an action to do	
        srv_info.request.agent = robotName_;
        srv_info.request.info ="ACTIONS_TODO";
        srv_info.request.actor = humanName_;
        if (client.call(srv_info)){
          if(srv_info.response.state == "READY"){//the robot thinks the human should act, it is necessary to solve the divergent belief
            srv_db.request.action = srv_info.response.action;
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
		  }
		}else{
         ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
		}
	  }
	}else{
     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
	}

	return "IDLE";
}

/*
State where the human is ACTING
*/
string HumanSM::actingState(string* object, bool* unexpected, string objectRobot, string robotState){

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

    lookAt(*object);
    if(robotState == "ACTING"){
        lookAt(objectRobot);
    }else{
        lookAtHuman();
    }

    ros::Publisher signal_pub = node_.advertise<head_manager::Signal>("head_manager/signal", 1000);
    head_manager::Signal msg;

    vector<string> objects;
    objects.push_back(*object);
    msg.entities = objects;
    vector<double> durations;
    durations.push_back(0.0);
    msg.durations = durations;
    msg.urgency = 0.8;
    msg.importancy = 0.8;
    msg.weight = 0.6;
    signal_pub.publish(msg);

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
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    ros::ServiceClient client_db = node_.serviceClient<supervisor_msgs::SolveDivergentBelief>("mental_state/solve_divergent_belief");
    supervisor_msgs::GetInfo srv_info;
    supervisor_msgs::SolveDivergentBelief srv_db;
    srv_info.request.info ="ACTIONS_TODO";
    srv_info.request.agent = humanName_;
    srv_info.request.actor = humanName_;
	srv_db.request.agent = humanName_;
    if (client.call(srv_info)){
     if(srv_info.response.state == "NEEDED"){
        //we verify that the robot also thinks the action is NEEDED
        srv_info.request.info ="ACTION_STATE";
        srv_info.request.agent = robotName_;
        srv_info.request.action = srv_info.response.action;
        if (client.call(srv_info)){
         if(srv_info.response.state == "NEEDED"){
			//the state is the same in the robot knowledge, the human continue to WAIT
			return "WAITING";
		 }else{//there is a divergent belief, we solve it and return to IDLE
            srv_db.request.action = srv_info.response.action;
			if (!client_db.call(srv_db)){
				ROS_ERROR("[state_machines] Failed to call service mental_state/solve_divergent_belief");
			}
			ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
			return "IDLE";
		  }
		}else{
         ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
		}
	 }else{//we return to IDLE to look for other actions
		ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
		return "IDLE";
	  }
	}else{
     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
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

    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    ros::ServiceClient action_state = node_.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
    supervisor_msgs::GetInfo srv_info;
    supervisor_msgs::ChangeState srv_action;
    srv_info.request.info ="ACTIONS_TODO";
    srv_info.request.agent = humanName_;
    srv_info.request.actor = humanName_;

    if(!timerStarted_ && robotState =="WAITING"){//we just enter the state, we start the timer
		start_ = clock();
		timerStarted_ = true;
        signalGiven_ = false;
    }else if(timerStarted_ && robotState !="WAITING"){
        timerStarted_ = false;
        signalGiven_ = false;
    }else{
        if (client.call(srv_info)){
            if(srv_info.response.state == "READY"){
                shouldDoAction_ = srv_info.response.action;
                pair<vector<string>, vector<double> > objects = signalObjects(srv_info.response.action);
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
                    srv_info.request.info ="ACTION_STATE";
                    srv_info.request.agent = robotName_;
                    srv_info.request.action = srv_info.response.action;
                    supervisor_msgs::Action actionTodo = srv_info.response.action;
                    if (client.call(srv_info)){
                     if(srv_info.response.state != "ASKED"){//if the action is not already ASKED, the robot asks to do the action
                        ros::ServiceClient client_ask = node_.serviceClient<supervisor_msgs::Ask>("dialogue_node/ask");
                        supervisor_msgs::Ask srv_ask;
                        srv_ask.request.type = "ACTION";
                        srv_ask.request.subType = "CAN";
                        srv_ask.request.action = actionTodo;
                        srv_ask.request.receiver = humanName_;
                      }else{//else, we consider the action failed
                         srv_action.request.type = "action";
                         srv_action.request.action = actionTodo;
                         srv_action.request.state = "FAILED";
                          if (!action_state.call(srv_action)){
                            ROS_ERROR("Failed to call service mental_state/change_state");
                          }
                       ROS_INFO("[state_machines] %s goes to IDLE", humanName_.c_str());
                       shouldDoAction_.name = "NULL";
                       return "IDLE";
                      }
                    }else{
                     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
                    }
                }else{
                 ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
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
Look at an object
*/
void HumanSM::lookAt(string object){

    //we get the coordonates of the object
    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == object){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
   }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read %s pose from toaster", object.c_str());
    }

    //we look at the object
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "map";
    goal.head_target_x = x;
    goal.head_target_y = y;
    goal.head_target_z = z;
    head_action_client->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout){
        ROS_INFO("[action_executor] pr2motion head action did not finish before the time out.");
    }
}

void HumanSM::lookAtHuman(){


    //we get the coordonates of the object
    toaster_msgs::HumanListStamped humanList;
    double x,y,z;
   try{
       humanList  = *(ros::topic::waitForMessage<toaster_msgs::HumanListStamped>("pdg/humanList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Human>::iterator it = humanList.humanList.begin(); it != humanList.humanList.end(); it++){
         if(it->meAgent.meEntity.id == "HERAKLES_HUMAN1"){
             for(std::vector<toaster_msgs::Joint>::iterator itt = it->meAgent.skeletonJoint.begin(); itt != it->meAgent.skeletonJoint.end(); itt++){
                if(itt->meEntity.id == "head"){
                    x = itt->meEntity.pose.position.x;
                    y = itt->meEntity.pose.position.y;
                    z = itt->meEntity.pose.position.z;
                }
             }
            break;
         }
       }
   }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read human pose from toaster");
    }

    //we look at the object
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "map";
    goal.head_target_x = x;
    goal.head_target_y = y;
    goal.head_target_z = z;
    head_action_client->sendGoal(goal);



    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout){
        ROS_INFO("[action_executor] pr2motion head action did not finish before the time out.");
    }

    ros::Publisher tag_detection_pub = node_.advertise <std_msgs::Bool>("ar_track_alvar/enable_detection",1);
    std_msgs::Bool msg;
    msg.data  = true;
    tag_detection_pub.publish(msg);
    ros::Duration(0.05).sleep();
    msg.data  = true;
    tag_detection_pub.publish(msg);



}


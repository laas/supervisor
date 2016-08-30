/**
author Sandra Devin

Main class of the action executor

This module allows the robot to execute high level actions.
Available actions:

In progess: pick, place (+pickandplace), drop (+pickanddrop)

TODO: placereachable (+pickandplacereachable), give/grab, moveTo, goTo (+engage/disengage), scan, sweep

**/

#include <action_executor/action_executor.h>

ros::NodeHandle* node_;
Connector* connector_;
string humanSafetyJoint, robotToaster;
double safetyThreshold;
ros::Publisher focus_pub;

ActionExecutor::ActionExecutor(string name):
action_server_(node_, name, 
	boost::bind(&ActionExecutor::execute,this, _1), false)
 {
 	action_server_.start();
 	ROS_INFO("[action_executor] Action server ready");
}


void ActionExecutor::execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal) {
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
    supervisor_msgs::ChangeState srv;
    srv.request.type= "action";
    srv.request.action = goal->action;

	//Getting action informations
    string name = goal->action.name;
    int id = goal->action.id;
	ROS_INFO("Executing action %s with id %i with parameters:", name.c_str(), id);
    for(int i=0; i<goal->action.parameters.size();i++){
        ROS_INFO("  %s", goal->action.parameters[i].c_str());
	}

	//Creating the action
	VirtualAction* act = NULL;
    act = initializeAction(goal->action);
	if(!act){
		ROS_WARN("Aborted");
		result_.report = false;
        result_.state = "NON_VALID";
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
		action_server_.setAborted(result_);
	   ROS_INFO("[action_executor] Action failed at creation");
		return;
	}

	//send the fact that the action is in progress to the MS Manager
	srv.request.state = "PROGRESS";
	if (!client.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
    }

	//Checking preconditions
	feedback_.state = "PREC";
	action_server_.publishFeedback(feedback_);
	if(!act->preconditions()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
         ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
		}
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        connector_->weightFocus_ = 0.0;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in preconditions");
		return;
	}

    if(action_server_.isPreemptRequested() || connector_->stopOrder_){
        result_.state = "PREEMPTED";
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        connector_->weightFocus_ = 0.0;
        action_server_.setPreempted(result_);
        ROS_INFO("[action_executor] Action stoped");
        return;
    }

	//Plan for the action
	feedback_.state = "PLAN";
	action_server_.publishFeedback(feedback_);
	if(!act->plan()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
         ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
		}
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        connector_->weightFocus_ = 0.0;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in planning");
		return;
    }

    if(action_server_.isPreemptRequested() || connector_->stopOrder_){
        result_.state = "PREEMPTED";
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        connector_->weightFocus_ = 0.0;
        action_server_.setPreempted(result_);
        ROS_INFO("[action_executor] Action stoped");
        return;
    }

	//Execution of the action
	feedback_.state = "EXEC";
	action_server_.publishFeedback(feedback_);
    if(!act->exec(&action_server_)){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
         ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
		}
        result_.report = false;
        connector_->weightFocus_ = 0.0;
        if(connector_->rightArmRestPose_ != connector_->rightArmPose_){
            result_.shouldRetractRight = true;
        }else{
            result_.shouldRetractRight = false;
        }
        if(connector_->leftArmRestPose_ != connector_->leftArmPose_){
            result_.shouldRetractLeft = true;
        }else{
            result_.shouldRetractLeft = false;
        }
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in execution");
		return;
	}

	//Apply/Check Post-conditions
	feedback_.state = "POST";
	action_server_.publishFeedback(feedback_);
	if(!act->post()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
         ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
		}
        result_.report = false;
        connector_->weightFocus_ = 0.0;
        if(connector_->rightArmRestPose_ != connector_->rightArmPose_){
            result_.shouldRetractRight = true;
        }else{
            result_.shouldRetractRight = false;
        }
        if(connector_->leftArmRestPose_ != connector_->leftArmPose_){
            result_.shouldRetractLeft = true;
        }else{
            result_.shouldRetractLeft = false;
        }
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in post conditions");
		return;
	}
	
    srv.request.action = act->getInstantiatedAction();
	srv.request.state = "DONE";
	if (!client.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service mental_state/change_state");
    }

    ros::ServiceClient clientPE = node_.serviceClient<supervisor_msgs::HumanAction>("plan_executor/robot_action");
    supervisor_msgs::HumanAction srvPE;
    srvPE.request.action = goal->action;
    if (!clientPE.call(srvPE)){
     ROS_ERROR("[action_executor] Failed to call service plan_executor/robot_action");
    }

    if(connector_->rightArmRestPose_ != connector_->rightArmPose_){
        result_.shouldRetractRight = true;
    }else{
        result_.shouldRetractRight = false;
    }
    if(connector_->leftArmRestPose_ != connector_->leftArmPose_){
        result_.shouldRetractLeft = true;
    }else{
        result_.shouldRetractLeft = false;
    }
    result_.report = true;
    connector_->weightFocus_ = 0.0;
	result_.state = "OK";
    action_server_.setSucceeded(result_);
	ROS_INFO("[action_executor] Action succeed");
}


VirtualAction* ActionExecutor::initializeAction(supervisor_msgs::Action action) {
	VirtualAction* act = NULL;

    connector_->stopOrder_ = false;

	if(action.name == "pick"){
		act = new Pick(action, connector_);
	}else if(action.name == "place"){
        act = new Place(action, connector_);
    }else if(action.name == "placereachable"){
        act = new PlaceReachable(action, connector_);
    }else if(action.name == "pickandplace"){
		act = new PickAndPlace(action, connector_);
    }else if(action.name == "pickandplacereachable"){
        act = new PickAndPlaceReachable(action, connector_);
    }else if(action.name == "drop"){
		act = new Drop(action, connector_);
	}else if(action.name == "pickanddrop"){
		act = new PickAndDrop(action, connector_);
	}else if(action.name == "moveTo"){
		act = new MoveTo(action, connector_);
    }else if(action.name == "scan"){
        act = new Scan(action, connector_);
    }else{
		ROS_WARN("[action_executor] Unknown action");
	}	

	return act;
}

/*
Service call when a stop order arrives
*/
bool stopOrder(supervisor_msgs::Empty::Request  &req, supervisor_msgs::Empty::Response &res){

   connector_->stopOrder_ = true;

    return true;
}

void agentFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){

    vector<toaster_msgs::Fact> distanceFacts = msg->factList;

    for(vector<toaster_msgs::Fact>::iterator it = distanceFacts.begin(); it != distanceFacts.end(); it++){
        if(it->property == "Distance"){
            if(it->subjectId == humanSafetyJoint && it->targetId == robotToaster){
                if(it->doubleValue < safetyThreshold){
                    connector_->stopOrder_ = true;
                }else{
                    connector_->stopOrder_ = false;
                }
            }
        }
    }
}

void publishFocus(){
    ros::Rate loop_rate(30);

    while (true) {
        supervisor_msgs::Focus msg;
        msg.object = connector_->objectFocus_;
        msg.weight = connector_->weightFocus_;
        msg.stopable = connector_->stopableFocus_;
        focus_pub.publish(msg);
        loop_rate.sleep();
    }

}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle node;
  node_ = &node;

  ROS_INFO("[action_executor] Init action_executor");

  node.getParam("humanSafetyJoint", humanSafetyJoint);
  node.getParam("safetyThreshold", safetyThreshold);
  node.getParam("robot/toasterName", robotToaster);
   
  Connector connector;
  connector_ = &connector;


  ActionExecutor executor("supervisor/action_executor");

  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1000, agentFactListCallback);

  ros::ServiceServer service_stop = node.advertiseService("action_executor/stop", stopOrder); //stop the execution

  focus_pub = node.advertise<supervisor_msgs::Focus>("action_executor/focus", 1000);

  boost::thread pubThread(publishFocus);

  ros::spin();

  return 0;
}

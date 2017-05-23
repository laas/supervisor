/**
author Sandra Devin

Simple dialogue node

**/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <pr2motion/connect_port.h>
#include <pr2motion/Head_Move_TargetAction.h>
#include <pr2motion/Z_Head_SetMinDuration.h>
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/FactList.h"

#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/String.h"

ros::NodeHandle* node_;
std::string robotName_;
actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>* head_action_client_;
std::string defaultTarget_, speakingTarget_, headJoint_, handJoint_;
std::string robotTarget_, currentTarget_;
std::vector<std::string> humanTargets_;
bool robotActing_, robotFocus_;
int robotActionId_;
bool shouldMove_;
std::vector<supervisor_msgs::Action> previousActions_;
bool isSpeaking_;
int oldRobotIdAction_;
bool lookMoving_, lookInArea_, lookAnticipation_, lookAction_, lookActionAtEnd_;
double distAnticipation_;
bool isMoving_, isInArea_;
std::string objectNear_;
double minXArea_, maxXArea_, minYArea_, maxYArea_;
bool shouldSignal_, needSignal_, signalGiven_;
std::string objectSignal_;
bool inAction_;
int nbNotMoving_ = 0;
int nbMaxNotMoving_ = 15;
int nbNear_ = 0;
int nbMaxNear_ = 5;

/**
 * \brief Init head management of pr2motion
 * */
void initPR2motion(){
    ros::Duration(1.0).sleep();
    head_action_client_ = new actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>("pr2motion/Head_Move_Target", true);
    head_action_client_->waitForServer();

    ROS_WARN("PR2motion server ok");

    ros::ServiceClient connect_port_srv = node_->serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    pr2motion::connect_port srv;
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect_port_srv.call(srv)){
      ROS_ERROR("[simple_head_manager] Failed to call service pr2motion/connect_port");
    }

    pr2motion::Z_Head_SetMinDuration srv_MinDuration;
    srv_MinDuration.request.head_min_duration=0.5;
    if(!ros::service::call("/pr2motion/Z_Head_SetMinDuration",srv_MinDuration))
        ROS_ERROR("[robot_observer] Failed to call service /pr2motion/Z_Head_SetMinDuration");
}

/**
 * \brief Ask pr2motion to look at a point
 * @param point the point to look at
 * */
void lookAtPoint(geometry_msgs::Point point){

    if(!shouldMove_){
        return;
    }

    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "/map";
    goal.head_target_x = point.x;
    goal.head_target_y = point.y;
    goal.head_target_z = point.z;

    head_action_client_->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client_->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout)
      ROS_INFO("[simple_head_manager] Action did not finish before the time out.");
}

/**
 * \brief Get the position of an object
 * @param object the object we want the position
 * @return a point corresponding to the object position
 * */
geometry_msgs::Point getPoseObject(std::string object){

    toaster_msgs::ObjectListStamped objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
      if(it->meEntity.id == object){
         return it->meEntity.pose.position;
      }
    }

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    return point;
}

/**
 * \brief Get the position of an agent
 * @param agent the agent we want the position
 * @return a point corresponding to the agent position
 * */
geometry_msgs::Point getPoseAgent(std::string joint){

    std::string agent = defaultTarget_;

    toaster_msgs::HumanListStamped humanList  = *(ros::topic::waitForMessage<toaster_msgs::HumanListStamped>("pdg/humanList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Human>::iterator it = humanList.humanList.begin(); it != humanList.humanList.end(); it++){
      if(it->meAgent.meEntity.id == agent){
         for(std::vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); itj++){
             if(itj->meEntity.id == joint){
                return itj->meEntity.pose.position;
             }
         }
      }
    }

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    return point;
}

std::string getHumanTarget(){

    if((isMoving_ && lookMoving_) && (!lookInArea_ || isInArea_)){
        return handJoint_;
    }else if((isInArea_ && lookInArea_) && (isMoving_ || !lookMoving_)){
        return handJoint_;
    }

    return headJoint_;
}

/**
 * \brief Get the point to look
 * @return a point to look at
 * */
geometry_msgs::Point getTarget(){

    if(robotActing_ && !inAction_){
        inAction_ = true;
        signalGiven_ = false;
    }else if(!robotActing_ && inAction_){
        inAction_ = false;
        if(shouldSignal_ && !signalGiven_){
            needSignal_ = true;
        }
    }

    if(isSpeaking_ && !robotFocus_){
        currentTarget_ = speakingTarget_;
        return getPoseAgent(currentTarget_);
    }else if(humanTargets_.size() > 0 && !robotFocus_ && lookAction_){
        currentTarget_ = humanTargets_[0];
        humanTargets_.erase(humanTargets_.begin());
        return getPoseObject(currentTarget_);
    }else if(robotActing_){
        if(robotTarget_ == defaultTarget_){
            currentTarget_ = robotTarget_;
            return getPoseAgent(currentTarget_);
        }else if(robotTarget_ == "SIGNAL"){
            needSignal_ = true;
            return getPoseObject(currentTarget_);
        }else{
            currentTarget_ = robotTarget_;
            return getPoseObject(currentTarget_);
        }
    }else if(lookActionAtEnd_ && humanTargets_.size() > 0){
        currentTarget_ = humanTargets_[0];
        humanTargets_.erase(humanTargets_.begin());
        return getPoseObject(currentTarget_);
    }else if(lookAnticipation_ && objectNear_ != "NULL"){
        currentTarget_  = objectNear_;
        return getPoseObject(currentTarget_);
    }else{
        currentTarget_ = getHumanTarget();
        return getPoseAgent(currentTarget_);
    }

    robotActing_ = false;
    isSpeaking_ = false;

    return getPoseAgent(defaultTarget_);
}


/**
 * \brief Callback of the robot action topic
 * @param msg topic msg
 * */
void robotActionCallback(const supervisor_msgs::Action::ConstPtr& msg){

    if(msg->headFocus.size() > 0 && oldRobotIdAction_ != msg->id){
        robotActing_ = true;
        robotActionId_ = msg->id;
        robotFocus_ = msg->shouldKeepFocus;
        robotTarget_ = msg->headFocus;
    }
}


/**
 * \brief Callback of the humans action topic
 * @param msg topic msg
 * */
void humansActionCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> actions = msg->actions;
    for(std::vector<supervisor_msgs::Action>::iterator it = actions.begin(); it != actions.end(); it++){
        humanTargets_.push_back(it->headFocus);
    }
}

/**
 * \brief Callback of the previous action topic
 * @param msg topic msg
 * */
void previousActionCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> newPrev = msg->actions;
    if(newPrev.size() > previousActions_.size()){
        for(int i = previousActions_.size(); i < newPrev.size(); i++){
            if(newPrev[i].id == robotActionId_){
                oldRobotIdAction_ = robotActionId_;
                robotActionId_ = -1;
                robotActing_ = false;
                break;
            }
        }
        previousActions_ = newPrev;
    }
}

/**
 * \brief Callback of the previous action topic
 * @param msg topic msg
 * */
void speakingCallback(const std_msgs::Bool::ConstPtr& msg){

    if(msg->data){
        isSpeaking_ = true;
    }
}


/**
 * \brief Callback for agent monitor facts
 * @param msg the list of facts
 * */
void agentFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){

    std::string object = "NULL";
    bool movingFact = false;
    std::vector<toaster_msgs::Fact> agentFacts = msg->factList;
    for(std::vector<toaster_msgs::Fact>::iterator it = agentFacts.begin(); it != agentFacts.end(); it++){
        if(it->property == "Distance"){
            if(it->subjectId == handJoint_ && it->subjectOwnerId == defaultTarget_ && it->targetId == "SCAN_AREA1" && it->targetId != "GREEN_CUBE1"){
                if(it->doubleValue < distAnticipation_){
                    object = it->targetId;
                }
            }
        }else if(it->property == "IsMoving"){
            if(it->subjectId == handJoint_ && it->subjectOwnerId == defaultTarget_){
                if(it->stringValue == "true"){
                    movingFact = true;
                }
            }
        }
    }

    if(!movingFact && isMoving_){
	if(nbNotMoving_ < nbMaxNotMoving_){
		nbNotMoving_++;
	}else{
             isMoving_ = false;
	}
    }else if(movingFact && !isMoving_){
	ROS_WARN("isMoving");
	isMoving_ = true;
	nbNotMoving_ = 0;
    }else if(movingFact){
	 nbNotMoving_ = 0;
    }

    if(object == "NULL" && objectNear_ != "NULL"){
        if(nbNear_ < nbMaxNear_){
                nbNear_++;
        }else{
             objectNear_ = "NULL";
        }
    }else if(object != "NULL" && objectNear_ == "NULL"){
        objectNear_ = object;
        nbNotMoving_ = 0;
    }else if(object != "NULL"){
         nbNotMoving_ = 0;
    }
    
}
/**
 * \brief Callback for agent monitor facts
 * @param msg the list of facts
 * */
void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg){

    isInArea_ = false;

    std::vector<toaster_msgs::Human> humanList  = msg->humanList;
    for(std::vector<toaster_msgs::Human>::iterator it = humanList.begin(); it != humanList.end(); it++){
      if(it->meAgent.meEntity.id == defaultTarget_){
         for(std::vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); itj++){
             if(itj->meEntity.id == handJoint_){
                if(itj->meEntity.pose.position.x > minXArea_ && itj->meEntity.pose.position.x < maxXArea_
                        && itj->meEntity.pose.position.y > minYArea_ && itj->meEntity.pose.position.y < maxYArea_){
                    isInArea_ = true;
		    ROS_WARN("in area");
		    ROS_WARN("hand pose: %f %f, min: %f %f, max: %f %f", itj->meEntity.pose.position.x, itj->meEntity.pose.position.y, minXArea_, minYArea_, maxXArea_, maxYArea_);
                }
                break;
             }
         }
         break;
      }
    }
}

/**
 * \brief Service to look at an object/agent
 * @param req name of the object
 * @param res empty result
 * @return true
 * */
bool lookAt(supervisor_msgs::String::Request  &req, supervisor_msgs::String::Response &res){

    geometry_msgs::Point point;
    if(req.data == "HERAKLES_HUMAN1"){
        point = getPoseAgent(req.data);
    }else{
        point = getPoseObject(req.data);
    }
    lookAtPoint(point);
}

void signal(){

    geometry_msgs::Point point;
    point = getPoseAgent(defaultTarget_);
    lookAtPoint(point);
    point = getPoseObject(objectSignal_);
    lookAtPoint(point);
    point = getPoseAgent(defaultTarget_);
    lookAtPoint(point);
    signalGiven_ = true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "simple_head_manager");
  ros::NodeHandle node;
  node_ = &node;
  node_->getParam("/robot/name", robotName_);
  node_->getParam("/simple_head_manager/defaultTarget", defaultTarget_);
  node_->getParam("/simple_head_manager/headJoint", headJoint_);
  node_->getParam("/simple_head_manager/handJoint", handJoint_);
  node_->getParam("/simple_head_manager/shouldMove", shouldMove_);
  node_->getParam("/simple_head_manager/lookMoving", lookMoving_);
  node_->getParam("/simple_head_manager/lookInArea", lookInArea_);
  node_->getParam("/simple_head_manager/lookAnticipation", lookAnticipation_);
  node_->getParam("/simple_head_manager/lookAction", lookAction_);
  node_->getParam("/simple_head_manager/lookActionAtEnd", lookActionAtEnd_);
  node_->getParam("/simple_head_manager/distAnticipation", distAnticipation_);
  node_->getParam("/simple_head_manager/minXArea", minXArea_);
  node_->getParam("/simple_head_manager/maxXArea", maxXArea_);
  node_->getParam("/simple_head_manager/minYArea", minYArea_);
  node_->getParam("/simple_head_manager/maxYArea", maxYArea_);
  node_->getParam("/simple_head_manager/shouldSignal", shouldSignal_);
  node_->getParam("/simple_head_manager/objectSignal", objectSignal_);


  ros::Rate loop_rate(30);

  initPR2motion();

  robotActionId_ = -1;
  isSpeaking_ = false;
  oldRobotIdAction_ = -1;
  signalGiven_ = false;
  inAction_ = false;

  ros::Subscriber sub_robot_action = node_->subscribe("/action_executor/current_robot_action", 1, robotActionCallback);
  ros::Subscriber sub_humans_action = node_->subscribe("/human_monitor/current_humans_action", 1, humansActionCallback);
  ros::Subscriber sub_prev_action = node_->subscribe("/supervisor/previous_actions", 1, previousActionCallback);
  ros::Subscriber sub_dialogue = node_->subscribe("/dialogue_node/isSpeaking", 1, speakingCallback);
  ros::Subscriber sub_agent = node_->subscribe("/agent_monitor/factList", 1, agentFactListCallback);
  ros::Subscriber sub_human = node_->subscribe("/pdg/humanList", 1, humanListCallback);

  ros::Publisher focus_pub = node_->advertise<std_msgs::String>("/simple_head_manager/focus", 1);

  ros::ServiceServer service_look_at = node_->advertiseService("/simple_head_manager/look_at", lookAt);

  ROS_INFO("[simple_head_manager] simple_head_manager ready");

  while (node.ok()) {
      ros::spinOnce();

      geometry_msgs::Point point = getTarget();
      if(needSignal_ && !signalGiven_){
          signal();
      }else{
        lookAtPoint(point);
      }
      std_msgs::String msg;
      msg.data = currentTarget_;
      focus_pub.publish(msg);

      loop_rate.sleep();
  }

  return 0;
}

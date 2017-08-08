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

#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/String.h"

ros::NodeHandle* node_;
std::string robotName_;
actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>* head_action_client_;
std::string defaultTarget_, speakingTarget_, headJoint_;
std::string robotTarget_, currentTarget_;
std::vector<std::string> humanTargets_;
bool robotActing_, robotFocus_;
int robotActionId_;
bool shouldMove_;
std::vector<supervisor_msgs::Action> previousActions_;
bool isSpeaking_;
int oldRobotIdAction_;

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

    bool finishedBeforeTimeout = head_action_client_->waitForResult(ros::Duration(3.0));

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
geometry_msgs::Point getPoseAgent(std::string agent){

    toaster_msgs::HumanListStamped humanList  = *(ros::topic::waitForMessage<toaster_msgs::HumanListStamped>("pdg/humanList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Human>::iterator it = humanList.humanList.begin(); it != humanList.humanList.end(); it++){
      if(it->meAgent.meEntity.id == agent){
         for(std::vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); itj++){
             if(itj->meEntity.id == headJoint_){
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

/**
 * \brief Get the point to look
 * @return a point to look at
 * */
geometry_msgs::Point getTarget(){

    if(isSpeaking_ && !robotFocus_){
        currentTarget_ = speakingTarget_;
        return getPoseAgent(currentTarget_);
    }else if(humanTargets_.size() > 0 && !robotFocus_){
        currentTarget_ = humanTargets_[0];
        humanTargets_.erase(humanTargets_.begin());
        return getPoseObject(currentTarget_);
    }else if(robotActing_){
        currentTarget_ = robotTarget_;
        return getPoseObject(currentTarget_);
    }else{
        currentTarget_ = defaultTarget_;
        return getPoseAgent(currentTarget_);
    }

    robotActing_ = false;
    isSpeaking_ = false;

    return getPoseAgent(currentTarget_);
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
        if(it->headFocus != "NONE"){
            humanTargets_.push_back(it->headFocus);
        }
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
    }else{
		isSpeaking_ = false;
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

int main (int argc, char **argv)
{
  ros::init(argc, argv, "simple_head_manager");
  ros::NodeHandle node;
  node_ = &node;
  node_->getParam("/robot/name", robotName_);
  node_->getParam("/simple_head_manager/defaultTarget", defaultTarget_);
  node_->getParam("/simple_head_manager/headJoint", headJoint_);
  node_->getParam("/simple_head_manager/shouldMove", shouldMove_);
  node_->getParam("/simple_head_manager/speakingTarget", speakingTarget_);
  ros::Rate loop_rate(30);

  initPR2motion();

  robotActionId_ = -1;
  isSpeaking_ = false;
  oldRobotIdAction_ = -1;

  ros::Subscriber sub_robot_action = node_->subscribe("/action_executor/current_robot_action", 1, robotActionCallback);
  ros::Subscriber sub_humans_action = node_->subscribe("/human_monitor/current_humans_action", 1, humansActionCallback);
  ros::Subscriber sub_prev_action = node_->subscribe("/supervisor/previous_actions", 1, previousActionCallback);
  ros::Subscriber sub_dialogue = node_->subscribe("/dialogue_node/isSpeaking", 1, speakingCallback);

  ros::Publisher focus_pub = node_->advertise<std_msgs::String>("/simple_head_manager/focus", 1);

  ros::ServiceServer service_look_at = node_->advertiseService("/simple_head_manager/look_at", lookAt);

  ROS_INFO("[simple_head_manager] simple_head_manager ready");

  while (node.ok()) {
      ros::spinOnce();

      geometry_msgs::Point point = getTarget();
      lookAtPoint(point);
      std_msgs::String msg;
      msg.data = currentTarget_;
      focus_pub.publish(msg);

      loop_rate.sleep();
  }

  return 0;
}

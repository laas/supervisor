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
#include <pr2motion/connect_port.h>
#include <pr2motion/Head_Move_TargetAction.h>
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/HumanListStamped.h"

using namespace std;

ros::NodeHandle* node;
string robotName_;
actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>* head_action_client;

void initPR2motion(){
    head_action_client = new actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>("pr2motion/Head_Move_Target", true);
    head_action_client->waitForServer();

    ros::ServiceClient connect_port_srv_ = node->serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    pr2motion::connect_port srv;
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect_port_srv_.call(srv)){
      ROS_ERROR("[simple_head_manager] Failed to call service pr2motion/connect_port");
    }
}

void lookAtPoint(geometry_msgs::Point point){

    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "/map";
    goal.head_target_x = point.x;
    goal.head_target_y = point.y;
    goal.head_target_z = point.z;

    head_action_client->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout)
      ROS_INFO("[simple_head_manager] Action did not finish before the time out.");
}

geometry_msgs::Point getPoseObject(string object){
    try{
        toaster_msgs::ObjectListStamped objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
        for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
          if(it->meEntity.id == object){
             return it->meEntity.pose.position;
          }
        }
    }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read %s pose from toaster", object.c_str());
    }

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    return point;
}

geometry_msgs::Point getPoseAgent(string agent){
    try{
        toaster_msgs::HumanListStamped humanList  = *(ros::topic::waitForMessage<toaster_msgs::HumanListStamped>("pdg/humanList",ros::Duration(1)));
        for(vector<toaster_msgs::Human>::iterator it = humanList.humanList.begin(); it != humanList.humanList.end(); it++){
          if(it->meAgent.meEntity.id == agent){
             for(vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); it++){
                 if(itj->meEntity.id == "rightHand"){
                    return itj->meEntity.pose.position;
                 }
             }
          }
        }
    }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read %s pose from toaster", agent.c_str());
    }

    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    return point;
}


void headFocus(const std_msgs::String::ConstPtr& msg){

    string target = msg->data;

    geometry_msgs::Point pose;
    if(target == "HERAKLES_HUMAN1"){
        pose = getPoseAgent(target);
    }else{
        pose = getPoseObject(target);
    }
    lookAtPoint(pose);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "simple_head_manager");
  ros::NodeHandle _node;
  node = &_node;
  node->getParam("/robot/name", robotName_);

  initPR2motion();

  ros::Subscriber subHeadFocus = node->subscribe("supervisor/head_focus", 1, headFocus);

  ROS_INFO("[simple_head_manager] plan_executor ready");

  ros::spin();

  return 0;
}

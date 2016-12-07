/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "supervisor_msgs/ActionExecutorAction.h"
#include "toaster_msgs/SetInfoDB.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/RemoveFromHand.h"
#include <pr2motion/Arm_Right_MoveToQGoalAction.h>
#include <pr2motion/Arm_Left_MoveToQGoalAction.h>

ros::NodeHandle* node_;
int nbTest = 50;
std::string logFilePath = "/home/sdevin/catkin_ws/src/supervisor/tests/logs/PickAndPlace.dat";
std::ofstream fileLog;
double x_min = 4.5;
double x_max = 4.9;
double y_min = 3.5;
double y_max = 4.1;
double z = 0.7;


/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_pickanplace");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[test_pickanplace] Init");

  //create the action server
  actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> actionClient("supervisor/action_executor", true);
  actionClient.waitForServer();

  //open the file
   fileLog.open(logFilePath.c_str(), std::ios::out|std::ios::trunc);

  ros::ServiceClient client = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
  ros::ServiceClient client_set_pose_ = node_->serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");
  ros::ServiceClient client_rm_ = node_->serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");

 actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveToQGoalAction>*PR2motion_arm_right_Q_ = new actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveToQGoalAction>("pr2motion/Arm_Right_MoveToQGoal",true);
 PR2motion_arm_right_Q_->waitForServer();
 actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveToQGoalAction>* PR2motion_arm_left_Q_ = new actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveToQGoalAction>("pr2motion/Arm_Left_MoveToQGoal",true);
 PR2motion_arm_left_Q_->waitForServer();

    //move arms
    pr2motion::Arm_Right_MoveToQGoalGoal goalQ;
    goalQ.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
    goalQ.shoulder_pan_joint = -1.952888;
    goalQ.shoulder_lift_joint = -0.095935;
    goalQ.upper_arm_roll_joint = -0.601572;
    goalQ.elbow_flex_joint = -1.600124;
    goalQ.forearm_roll_joint = 0.018247;
    goalQ.wrist_flex_joint = -0.432897;
    goalQ.wrist_roll_joint = -1.730082;\
    PR2motion_arm_right_Q_->sendGoal(goalQ);
    ROS_INFO("[action_manager] Waiting for arms move");
    bool finishedBeforeTimeout = PR2motion_arm_right_Q_->waitForResult(ros::Duration(300.0));
    if (!finishedBeforeTimeout){
    ROS_INFO("Action PR2 go to Q did not finish before the time out.");
    }
    pr2motion::Arm_Left_MoveToQGoalGoal goalQL;
    goalQL.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
    goalQL.shoulder_pan_joint = 1.91155;
    goalQL.shoulder_lift_joint = -0.0984492;
    goalQL.upper_arm_roll_joint = 0.6;
    goalQL.elbow_flex_joint = -1.6534;
    goalQL.forearm_roll_joint = -0.02173;
    goalQL.wrist_flex_joint = -0.473717;
    goalQL.wrist_roll_joint = -1.76561;\
    PR2motion_arm_left_Q_->sendGoal(goalQL);
    ROS_INFO("[action_manager] Waiting for arms move");
    finishedBeforeTimeout = PR2motion_arm_left_Q_->waitForResult(ros::Duration(300.0));
    if (!finishedBeforeTimeout){
    ROS_INFO("Action PR2 go to Q did not finish before the time out.");
    }

  ROS_INFO("[test_pickanplace] Ready");

  int i = 0;
  while(node.ok() & i < nbTest){
      supervisor_msgs::ActionExecutorGoal goal;
      supervisor_msgs::Action action;
      //place the cube at a random place in the table
      double x = (rand()%(int)((x_max - x_min)*100) + x_min*100)/100;
      double y = (rand()%(int)((y_max - y_min)*100) + y_min*100)/100;
      toaster_msgs::SetEntityPose srv;
      srv.request.id = "LOTR_TAPE";
      srv.request.type = "object";
      srv.request.pose.position.x = x;
      srv.request.pose.position.y = y;
      srv.request.pose.position.z = z;
      srv.request.pose.orientation.x = 0.0;
      srv.request.pose.orientation.y = 0.0;
      srv.request.pose.orientation.z = 0.0;
      srv.request.pose.orientation.w = 1.0;
      if (!client_set_pose_.call(srv)){
       ROS_ERROR("Failed to call service toaster_simu/set_entity_pose");
      }

      //ask for a pick
      action.name = "pick";
      action.actors.push_back("PR2_ROBOT");
      action.parameter_keys.push_back("object");
      action.parameter_values.push_back("LOTR_TAPE");
      goal.action = action;
      actionClient.sendGoal(goal);
      bool finish = actionClient.waitForResult(ros::Duration(300.0));
      if(finish){
          std::string res;
          if(actionClient.getResult()->report){
              res = "OK";
          }else{
              res = "FAILED";
          }
          fileLog << "pick" << " " << actionClient.getResult()->timeTot << " " << actionClient.getResult()->timeDB << " " <<
                  actionClient.getResult()->timeToaster << " " << actionClient.getResult()->timePlan << " " <<
                  actionClient.getResult()->timeGTP << " " << actionClient.getResult()->timeExec << " " << res.c_str() << std::endl;
          if(actionClient.getResult()->report){
              //add effects to the DB
              std::vector<toaster_msgs::Fact> facts;
              toaster_msgs::Fact fact;
              fact.subjectId = "LOTR_TAPE";
              fact.property = "isHoldBy";
              fact.targetId = "PR2_ROBOT";
              facts.push_back(fact);
              toaster_msgs::SetInfoDB srv;
              srv.request.agentId = "PR2_ROBOT";
              srv.request.facts = facts;
              srv.request.infoType = "FACT";
              srv.request.add = true;
              if (!client.call(srv)){
               ROS_ERROR("[action_manager] Failed to call service database_manager/set_info");
              }
              //if sucess ask for a place
              action.name = "place";
              action.parameter_keys.push_back("support");
              action.parameter_values.push_back("TABLE_4");
              goal.action = action;
              actionClient.sendGoal(goal);
              bool finish = actionClient.waitForResult(ros::Duration(300.0));
              if(finish){
                  if(actionClient.getResult()->report){
                      res = "OK";
                  }else{
                      res = "FAILED";
                  }
                  fileLog << "place" << " " << actionClient.getResult()->timeTot << " " << actionClient.getResult()->timeDB << " " <<
                          actionClient.getResult()->timeToaster << " " << actionClient.getResult()->timePlan << " " <<
                          actionClient.getResult()->timeGTP << " " << actionClient.getResult()->timeExec << " " << res.c_str() << std::endl;
                  if(!actionClient.getResult()->report){
                      //remove the object from hand if no success
                      toaster_msgs::RemoveFromHand srv_rm;
                      srv_rm.request.objectId = "LOTR_TAPE";
                      if (!client_rm_.call(srv_rm)){
                       ROS_ERROR("Failed to call service pdg/remove_from_hand");
                      }
                  }
              }else{
                  ROS_ERROR("[TESTS] Action Executor does not respond!");
              }
              //add effects to the DB
              srv.request.add = false;
              if (!client.call(srv)){
               ROS_ERROR("[action_manager] Failed to call service database_manager/set_info");
              }
          }else{
              ROS_WARN("[TESTS] Pick fails!");
          }
      }else{
          ROS_ERROR("[TESTS] Action Executor does not respond!");
      }

      ros::spinOnce();
      loop_rate.sleep();
      i++;
  }

  //close the file
  fileLog.close();

  return 1;
}

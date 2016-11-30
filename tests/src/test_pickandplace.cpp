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

ros::NodeHandle* node_;
int nbTest = (50);
std::string logFilePath = "/home/sdevin/catkin_ws/src/supervisor/tests/logs/PickAndPlace.dat";
std::ofstream fileLog;


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


  ROS_INFO("[test_pickanplace] Ready");

  int i = 0;
  while(node.ok() & i < nbTest){
      supervisor_msgs::ActionExecutorGoal goal;
      supervisor_msgs::Action action;

      //ask for a pick
      action.name = "pick";
      action.actors.push_back("PR2_ROBOT");
      action.parameter_keys.push_back("object");
      action.parameter_values.push_back("RED_CUBE");
      goal.action = action;
      actionClient.sendGoal(goal);
      bool finish = actionClient.waitForResult(ros::Duration(300.0));
      if(finish){
          fileLog << "pick" << " " << actionClient.getResult()->timeTot << " " << actionClient.getResult()->timeDB << " " <<
                  actionClient.getResult()->timeToaster << " " << actionClient.getResult()->timePlan << " " <<
                  actionClient.getResult()->timeGTP << " " << actionClient.getResult()->timeExec << std::endl;
          if(actionClient.getResult()->report){
              //if sucess ask for a place
              action.name = "place";
              action.parameter_keys.push_back("support");
              action.parameter_values.push_back("TABLE_4");
              goal.action = action;
              actionClient.sendGoal(goal);
              bool finish = actionClient.waitForResult(ros::Duration(300.0));
              if(finish){
                  fileLog << "place" << " " << actionClient.getResult()->timeTot << " " << actionClient.getResult()->timeDB << " " <<
                          actionClient.getResult()->timeToaster << " " << actionClient.getResult()->timePlan << " " <<
                          actionClient.getResult()->timeGTP << " " << actionClient.getResult()->timeExec << std::endl;
              }else{
                  ROS_ERROR("[TESTS] Action Executor does not respond!");
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

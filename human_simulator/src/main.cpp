/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <stdlib.h>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "toaster_msgs/ExecuteDB.h"
#include "supervisor_msgs/HumanAction.h"
#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/GoalsList.h"


ros::NodeHandle* node_;
std::string robotName_, mode_, humanState_, humanName_, robotObject_, pickedObject_, tempPickedObject_;
ros::ServiceClient client_human_action_;
ros::ServiceClient client_db_execute_;
ros::Publisher answer_pub_;
std::vector<supervisor_msgs::Action> oldMsgPrevious_;
std::ofstream logFile_;
int nbConflict_;
std::map<std::string, bool> objectsScanned;
std::map<std::string, std::string> objectsDecision, objectsColors;
int nbStack_;
std::string isOnScan1_, isOnScan2_;
bool started_;
int nbWait_;
double timeWait_;
bool stubborn_;
ros::Time startWaiting_;
bool isWaiting_;
std::vector<std::string> stack;
int nbExp;

/**
 * \brief Callback for asked actions by the robot
 * @param msg topic msg
 * */
void askCallback(const supervisor_msgs::Action::ConstPtr& msg){

    std_msgs::Bool ans;

    //find the concerned object
    supervisor_msgs::Action actionAsked = *msg;
    std::string object;
    for(int j = 0; j < actionAsked.parameter_keys.size(); j++){
        if(actionAsked.parameter_keys[j] == "object"){
            object = actionAsked.parameter_values[j];
        }
    }

    if(mode_ == "none"){
        ans.data = false;
        objectsDecision[object] = "NO";
        ROS_INFO("[HUMAN] answer no");
    }else if(mode_ == "all"){
        ans.data = true;
        objectsDecision[object] = "YES";
        ROS_INFO("[HUMAN] answer yes");
    }else{
        int nb = rand() % (100);
        if(nb < 50){
            ans.data = true;
            objectsDecision[object] = "YES";
            ROS_INFO("[HUMAN] answer yes");
        }else{
            ans.data = false;
            objectsDecision[object] = "NO";
            ROS_INFO("[HUMAN] answer no");
        }
    }

    ros::Duration(0.5).sleep();
    answer_pub_.publish(ans);
}

/**
 * \brief Callback for informed actions by the robot
 * @param msg topic msg
 * */
void informCallback(const supervisor_msgs::Action::ConstPtr& msg){

    supervisor_msgs::Action action = *msg;

    if(action.name == "pickanddrop"){
        std::string object;
        for(int j = 0; j < action.parameter_keys.size(); j++){
            if(action.parameter_keys[j] == "object"){
                object = action.parameter_values[j];
            }
        }
        if(action.actors[0] == robotName_){
            //the human deduce the object is scan if it does not know it
            objectsScanned[object] = true;
            if(!stubborn_){
                objectsDecision[object] = "NO";
            }
        }else if(action.actors[0] == humanName_){
            //the human deduce the object is scan if it does not know it
            objectsScanned[object] = true;
            if(!stubborn_){
                objectsDecision[object] = "YES";
            }
            if(isWaiting_ && (objectsDecision[object] == "NO" || mode_ == "none")){
                //change of decision
                nbWait_++;
            }
        }else{
            ROS_WARN("[human_simulator] wrong action actor!");
        }
    }
}


/**
 * \brief Callback for informed actions by the robot
 * @param msg topic msg
 * */
void informScanCallback(const std_msgs::String::ConstPtr& msg){

    objectsScanned[msg->data] = true;
}



/**
 * \brief Callback of the previous action topic
 * @param msg topic msg
 * */
void previousCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> newPrev = msg->actions;
    if(newPrev.size() > oldMsgPrevious_.size()){
        for(int i = oldMsgPrevious_.size(); i < newPrev.size(); i++){
            if(newPrev[i].actors[0] == robotName_ && newPrev[i].succeed){
                if(newPrev[i].name == "pickandplace" || newPrev[i].name == "place"){
                    //get object and support
                    std::string object, support;
                    for(int j = 0; j < newPrev[i].parameter_keys.size(); j++){
                        if(newPrev[i].parameter_keys[j] == "object"){
                            object = newPrev[i].parameter_values[j];
                        }
                        if(newPrev[i].parameter_keys[j] == "support"){
                            support = newPrev[i].parameter_values[j];
                        }
                    }
                    if(support == "SCAN_AREA1"){
                        isOnScan1_ = object;
                    }
                    if(support == "SCAN_AREA2"){
                        isOnScan2_ = object;
                    }
                }else if(newPrev[i].name == "scan" && humanState_ == "TABLE"){
                    //get object and support
                    std::string object;
                    for(int j = 0; j < newPrev[i].parameter_keys.size(); j++){
                        if(newPrev[i].parameter_keys[j] == "object"){
                            object = newPrev[i].parameter_values[j];
                        }
                    }
                    objectsScanned[object] = true;
                }
            }
        }
        oldMsgPrevious_ = newPrev;
    }

}

void resetEnv(){

    std::vector<std::string> objects;
    node_->getParam("human_simulator/objects", objects);

    humanState_ = "TABLE";
    pickedObject_ = "NONE";
    tempPickedObject_ = "NONE";
    for(std::vector<std::string>::iterator it = objects.begin(); it != objects.end(); it++){
        objectsScanned[*it] = false;
        objectsDecision[*it] = "NONE";
    }
    nbStack_ = 5;
    isOnScan1_ = "NONE";
    isOnScan2_ = "NONE";
    nbWait_ = 0;
    timeWait_ = 0.0;
    isWaiting_ = false;


    std::vector<std::string> stackHuman;
    std::ostringstream strs;
    strs << nbExp;
    std::string nbString = strs.str();
    std::string topic = "scan_simu" + nbString + "/stackHuman";
    node_->getParam(topic, stackHuman);
    for(std::vector<std::string>::iterator it = stackHuman.begin(); it != stackHuman.end(); it++){
        stack.push_back(*it);
    }
    nbExp++;
}


/**
 * \brief Callback of the goal list
 * @param msg topic msg
 * */
void goalsListCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    if(msg->changed){
        if(msg->currentGoal == "SCAN_US" && !started_){
            started_ = true;
        }
        if(msg->currentGoal != "SCAN_US" && started_){
            started_ = false;
            //we log results
            std::ofstream fileSave;
            //we save the execution time
            std::string fileName = "/home/sdevin/catkin_ws/src/supervisor/logs/Humans.txt";
            fileSave.open(fileName.c_str(), std::ios::app);
            std::ostringstream strs, strs2;
            strs << nbWait_;
            strs2 << timeWait_;
            std::string nbW = strs.str();
            std::string timeW = strs2.str();
            fileSave << "\t Nb wait: " << nbW.c_str() << "\t time wait: " << timeW.c_str() << std::endl;
            resetEnv();
        }
    }
}

/**
 * \brief Callback for pick actions of the robot
 * @param msg topic msg
 * */
void pickCallback(const std_msgs::String::ConstPtr& msg){

    std::string robotObject = msg->data;
    if(robotObject == isOnScan1_){
        isOnScan1_ = "NONE";
    }
    if(robotObject == isOnScan2_){
        isOnScan2_ = "NONE";
    }
}

void pickStack(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }


   pickedObject_ = stack.front();
   stack.erase(stack.begin());
   nbStack_ --;
   ROS_INFO("[human_simulator] Human pick stack");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "pickStack";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void pickArea1(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }

   if(pickedObject_ != "NONE"){
       tempPickedObject_ = isOnScan1_;
   }else{
       pickedObject_ = isOnScan1_;
   }

   isOnScan1_ = "NONE";
   ROS_INFO("[human_simulator] Human pick area1");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "pickArea1";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void pickArea2(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }

    if(pickedObject_ != "NONE"){
        tempPickedObject_ = isOnScan2_;
    }else{
        pickedObject_ = isOnScan2_;
    }

   isOnScan2_ = "NONE";
   ROS_INFO("[human_simulator] Human pick area2");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "pickArea2";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void placeArea1(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }

   isOnScan1_ = pickedObject_;
   if(tempPickedObject_ != "NONE"){
       pickedObject_ = tempPickedObject_;
       tempPickedObject_ = "NONE";
   }else{
       pickedObject_ = "NONE";
   }
   ROS_INFO("[human_simulator] Human place area1");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "placeArea1";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void placeArea2(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }

   isOnScan2_ = pickedObject_;
   if(tempPickedObject_ != "NONE"){
       pickedObject_ = tempPickedObject_;
       tempPickedObject_ = "NONE";
   }else{
       pickedObject_ = "NONE";
   }
   ROS_INFO("[human_simulator] Human place area2");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "placeArea2";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void dropGreen(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }

   pickedObject_ = "NONE";
   ROS_INFO("[human_simulator] Human drop green");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "dropGreen";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void dropRed(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }

   pickedObject_ = "NONE";
   ROS_INFO("[human_simulator] Human drop red");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "dropRed";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }
}

void moveToTable(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }
    ROS_INFO("[human_simulator] Human move to table");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "moveToTable";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }

   humanState_ = "TABLE";

   ros::Duration(2.0).sleep();
}

void moveToStack(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }
    ROS_INFO("[human_simulator] Human move to stack");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "moveToStack";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }

   humanState_ = "STACK";
}

void moveToBox(){

    if(isWaiting_){
        isWaiting_ = false;
        ros::Time now = ros::Time::now();
        ros::Duration d = now - startWaiting_;
        timeWait_ = timeWait_ + d.toSec();
    }
    ROS_INFO("[human_simulator] Human move to box");

   //call human monitor
   supervisor_msgs::Action action;
   action.actors.push_back(humanName_);
   action.name = "moveToBox";
   supervisor_msgs::HumanAction srv;
   srv.request.agent = humanName_;
   srv.request.action = action;
   if (!client_human_action_.call(srv)){
       ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
   }

   humanState_ = "BOX";
}





/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_simulator");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[human_simulator] Init");

  node_->getParam("supervisor/robot/name", robotName_);
  node_->getParam("human_simulator/mode", mode_);
  node_->getParam("human_simulator/stubborn", stubborn_);
  node_->getParam("human_simulator/humanName", humanName_);
  std::vector<std::string> objects;
  node_->getParam("human_simulator/objects", objects);
  node_->getParam("supervisor/nbStart", nbExp);

  int seed;
  node_->getParam("scan_simu1/seed", seed);
  srand (seed);

  std::string filePath = "/home/sdevin/catkin_ws/src/supervisor/logs/Human.txt";
  //logFile_.open(filePath.c_str() , std::ios::app);

  for(std::vector<std::string>::iterator it = objects.begin(); it != objects.end(); it++){
      std::string colorTopic = "scan_simu1/color/" + *it;
      node_->getParam(colorTopic, objectsColors[*it]);
  }

  resetEnv();
  started_ = false;

  client_human_action_ = node_->serviceClient<supervisor_msgs::HumanAction>("human_monitor/human_action_simu");
  client_db_execute_ = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");

  ros::Subscriber sub_previous = node_->subscribe("supervisor/previous_actions", 10, previousCallback);
  ros::Subscriber sub_ask = node_->subscribe("dialogue_node/asked_action", 10, askCallback);
  ros::Subscriber sub_inform = node_->subscribe("dialogue_node/inform_action", 10, informCallback);
  ros::Subscriber sub_inform_scan = node_->subscribe("dialogue_node/inform_scan", 10, informScanCallback);
  ros::Subscriber sub_goal = node.subscribe("/goal_manager/goalsList", 10, goalsListCallback);
  ros::Subscriber sub_pick = node.subscribe("/action_executor/pick", 10, pickCallback);

  answer_pub_ = node.advertise<std_msgs::Bool>("graphical_interface/boolAnswer", 10);

  ROS_INFO("[human_simulator] Ready");

  while(node.ok()){
      //activate the readers
      ros::spinOnce();

      bool isWaitingLoop = false;
      if(started_){
          if(humanState_ == "TABLE"){
              if(pickedObject_ != "NONE"){
                  //look for a placement
                  if(isOnScan2_ == "NONE"){
                      //Place
                      placeArea2();
                  }else if(isOnScan1_ == "NONE"){
                      //Place
                      placeArea1();
                  }
              }

              if(isOnScan2_ != "NONE" && objectsScanned[isOnScan2_] && objectsColors[isOnScan2_] == "green"){
              //look for green cubes to throw
                //Pick and drop
                    //Place if needed
                      if(pickedObject_ != "NONE"){
                          //Place
                          pickArea2();
                          placeArea2();
                      }else{
                          pickArea2();
                      }
                    moveToBox();
              }else if(isOnScan1_ != "NONE" && objectsScanned[isOnScan1_] && objectsColors[isOnScan1_] == "green"){
                //Pick and drop
                  //Place if needed
                    if(pickedObject_ != "NONE"){
                        //Place
                        pickArea1();
                        placeArea1();
                    }else{

                        pickArea1();
                    }
                    moveToBox();
              }else if(isOnScan2_ != "NONE" && objectsScanned[isOnScan2_] && objectsColors[isOnScan2_] == "red" && objectsDecision[isOnScan2_] != "NO"){
                //look for red cubes to throw
                if(objectsDecision[isOnScan2_] == "YES"){
                    //Pick and Drop
                    //Place if needed
                      if(pickedObject_ != "NONE"){
                          //Place
                          pickArea2();
                          placeArea2();
                      }else{

                          pickArea2();
                      }
                    moveToBox();
                }else if(objectsDecision[isOnScan2_] != "NO"){
                    if(mode_ == "none"){
                        objectsDecision[isOnScan2_] = "NO";
                    }else if(mode_ == "all"){
                        objectsDecision[isOnScan2_] = "YES";
                    }else{
                    //Decision
                        int nb = rand() % (100);
                        if(nb < 50){
                            objectsDecision[isOnScan2_] = "YES";
                        }else{
                            objectsDecision[isOnScan2_] = "NO";
                        }
                    }
                    if(objectsDecision[isOnScan2_] == "YES"){
                        //Pick and Drop
                        //Place if needed
                          if(pickedObject_ != "NONE"){
                              //Place
                              pickArea2();
                              placeArea2();
                          }else{

                              pickArea2();
                          }
                        moveToBox();
                    }
                }
              }else if(isOnScan1_ != "NONE" && objectsScanned[isOnScan1_] && objectsColors[isOnScan1_] == "red" && objectsDecision[isOnScan1_] != "NO"){
                  if(objectsDecision[isOnScan1_] == "YES"){
                      //Pick and Drop
                      //Place if needed
                        if(pickedObject_ != "NONE"){
                            //Place
                            pickArea1();
                            placeArea1();
                        }else{

                            pickArea1();
                        }
                      moveToBox();
                  }else if(objectsDecision[isOnScan1_] != "NO"){
                      if(mode_ == "none"){
                          objectsDecision[isOnScan1_] = "NO";
                      }else if(mode_ == "all"){
                          objectsDecision[isOnScan1_] = "YES";
                      }else{
                      //Decision
                          int nb = rand() % (100);
                          if(nb < 50){
                              objectsDecision[isOnScan1_] = "YES";
                          }else{
                              objectsDecision[isOnScan1_] = "NO";
                          }
                      }
                      if(objectsDecision[isOnScan1_] == "YES"){
                          //Pick and Drop
                          //Place if needed
                            if(pickedObject_ != "NONE"){
                                //Place
                                pickArea1();
                                placeArea1();
                            }else{

                                pickArea1();
                            }
                          moveToBox();
                      }
                  }
              }else if(nbStack_ > 0 && pickedObject_ == "NONE"){
                  //Go for an object
                  moveToStack();
              }else{
                  //Wait
                  if(!isWaiting_){
                      isWaiting_ = true;
                      startWaiting_ = ros::Time::now();
                  }
              }
          }else if(humanState_ == "STACK"){
              //Pick
              pickStack();
              //Go to table
              moveToTable();
          }else if(humanState_ == "BOX"){
              if(objectsColors[pickedObject_] == "green"){
                  //drop green
                  dropGreen();
              }else if(objectsColors[pickedObject_] == "red"){
                  //drop red
                  dropRed();
              }
              //if object to pick
              if(nbStack_ > 0){
                  //Go to stack
                  moveToStack();
              }else{
                  //Go to table
                  moveToTable();
              }
          }

      }

      loop_rate.sleep();
  }
}

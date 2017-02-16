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
std::string robotName_, mode_, humanState_, currentObject_, humanName_, robotObject_, pickedObject_, highLevelObject_;
std::vector<std::string> objects_, supports_;
bool isDoubleObject_, shouldStart_;
double pickTime_, placeTime_, answerTime_;
ros::ServiceClient client_human_action_;
ros::ServiceClient client_db_execute_;
ros::Time begin_;
std::map<std::string, std::string> highLevelNames_;
std::map<std::string, std::vector<std::string> > highLevelRefinment_;
ros::Publisher pick_pub_;
ros::Publisher answer_pub_;
std::vector<supervisor_msgs::Action> oldMsgPrevious_;
std::ofstream logFile_;
int nbConflict_;
std::string nbExp_;


/**
 * \brief Fill the highLevelNames map from param
 * */
void fillHighLevelNames(){

    std::vector<std::string> manipulableObjects;
    node_->getParam("/entities/objects", manipulableObjects);
    for(std::vector<std::string>::iterator it = manipulableObjects.begin(); it != manipulableObjects.end(); it++){
        highLevelNames_[*it] = *it;
        std::string paramName = "/entities/highLevelName/" + *it;
        if(node_->hasParam(paramName)){
            node_->getParam(paramName, highLevelRefinment_[*it]);
        }
    }

    std::vector<std::string> supportObjects;
    node_->getParam("/entities/supports", supportObjects);
    for(std::vector<std::string>::iterator it = supportObjects.begin(); it != supportObjects.end(); it++){
        highLevelNames_[*it] = *it;
        std::string paramName = "/entities/highLevelName/" + *it;
        if(node_->hasParam(paramName)){
            node_->getParam(paramName, highLevelRefinment_[*it]);
        }
    }

    std::vector<std::string> containerObjects;
    node_->getParam("/entities/containers", containerObjects);
    for(std::vector<std::string>::iterator it = containerObjects.begin(); it != containerObjects.end(); it++){
        highLevelNames_[*it] = *it;
        std::string paramName = "/entities/highLevelName/" + *it;
        if(node_->hasParam(paramName)){
            node_->getParam(paramName, highLevelRefinment_[*it]);
        }
    }

    for(std::map<std::string, std::vector<std::string> >::iterator it = highLevelRefinment_.begin(); it != highLevelRefinment_.end(); it++){
        for(std::vector<std::string>::iterator ith = it->second.begin(); ith != it->second.end(); ith++){
            highLevelNames_[*ith] = it->first;
        }
    }
}

/**
 * \brief Find refinment for an object to pick
 * @param agent the agent who should pick
 * @param object the object
 * @return the refinment
 * */
std::string findObjectRefinment(std::string agent, std::string object){

    std::vector<toaster_msgs::Fact> toCheck;

    if(object == "RED_CUBE" && !isDoubleObject_){
        return "NONE";
    }

    if(highLevelRefinment_[object].size() == 0){
        //already refined
        toaster_msgs::Fact fact;
        fact.subjectId = "NULL";
        fact.property = "isOn";
        fact.targetId = object;
        toCheck.push_back(fact);
        fact.subjectId = object;
        fact.property = "isReachableBy";
        fact.targetId = agent;
        toCheck.push_back(fact);
    }

    for(std::vector<std::string>::iterator it = highLevelRefinment_[object].begin(); it != highLevelRefinment_[object].end(); it++){
        //we check the condition for the object
        toaster_msgs::Fact fact;
        fact.subjectId = "NULL";
        fact.property = "isOn";
        fact.targetId = *it;
        toCheck.push_back(fact);
        fact.subjectId = *it;
        fact.property = "isReachableBy";
        fact.targetId = agent;
        toCheck.push_back(fact);
    }

    std::vector<std::string> inDB;
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.type = "INDIV";
    srv.request.agent = robotName_;
    srv.request.facts = toCheck;
    if (client_db_execute_.call(srv)){
        inDB = srv.response.results;
    }else{
       ROS_ERROR("[action_executor] Failed to call service database_manager/execute");
       return "NONE";
    }

     if(highLevelRefinment_[object].size() == 0){
         for(int i = 0; i < 2; i++){
             if(inDB[i] == "false"){
                 return "NONE";
             }
         }
         return object;
     }

    int c = 0;
    for(std::vector<std::string>::iterator it = highLevelRefinment_[object].begin(); it != highLevelRefinment_[object].end(); it++){
        //if the preconditions are checked for this object
        bool ok = true;
        for(int i = c; i < c + 2; i++){
            if(inDB[i] == "false"){
                ok = false;
                break;
            }
        }
        c = c + 2;
        if(ok){
            return *it;
        }
    }

    return "NONE";

}

/**
 * \brief Find refinment for a support to pick
 * @param agent the agent who should place
 * @param support the support
 * @return the refinment
 * */
std::string findSupportRefinment(std::string support){

    std::vector<toaster_msgs::Fact> toCheck;

    if(highLevelRefinment_[support].size() == 0){
        return support;
    }

    if(support == "PLACEMAT"){
        return "PLACEMAT_1";
    }

    for(std::vector<std::string>::iterator it = highLevelRefinment_[support].begin(); it != highLevelRefinment_[support].end(); it++){
        //we check the condition for the object
        toaster_msgs::Fact fact;
        fact.subjectId = "NULL";
        fact.property = "isOn";
        fact.targetId = *it;
        toCheck.push_back(fact);
        fact.subjectId = *it;
        fact.property = "isReachableBy";
        fact.targetId = robotName_;
        toCheck.push_back(fact);
        fact.subjectId = *it;
        fact.property = "isReachableBy";
        fact.targetId = humanName_;
        toCheck.push_back(fact);
    }

    std::vector<std::string> inDB;
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.type = "INDIV";
    srv.request.agent = robotName_;
    srv.request.facts = toCheck;
    if (client_db_execute_.call(srv)){
        inDB = srv.response.results;
    }else{
       ROS_ERROR("[action_executor] Failed to call service database_manager/execute");
       return "NONE";
    }

    int c = 0;
    for(std::vector<std::string>::iterator it = highLevelRefinment_[support].begin(); it != highLevelRefinment_[support].end(); it++){
        //if the preconditions are checked for this object
        bool ok = true;
        for(int i = c; i < c + 3; i++){
            if(inDB[i] == "false"){
                ok = false;
                break;
            }
        }
        c = c + 3;
        if(ok){
            return *it;
        }
    }

    return "NONE";

}


/**
 * \brief Place an object
 * @param object the object
 * */
void placeObject(std::string object){

    supervisor_msgs::Action action;
    action.actors.push_back(humanName_);
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(object);
    if(object == "STICK"){
        action.name = "placeStick";
        action.parameter_keys.push_back("support1");
        action.parameter_values.push_back("RED_CUBE1");
        action.parameter_keys.push_back("support2");
        action.parameter_values.push_back("RED_CUBE2");
    }else{
        std::string support = supports_.front();
        support = findSupportRefinment(support);
        if(support == "NONE"){
            ROS_ERROR("[human_simulator] No possible support");
            return;
        }
        action.name = "place";
        action.parameter_keys.push_back("support");
        action.parameter_values.push_back(support);
    }

    //execute the pick
    supervisor_msgs::HumanAction srv;
    srv.request.agent = humanName_;
    srv.request.action = action;
    if (!client_human_action_.call(srv)){
        ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
    }

    //we update the lists
    pickedObject_ = "NONE";
    supports_.erase(supports_.begin());
    objects_.erase(objects_.begin());
    humanState_ = "IDLE";


}


/**
 * \brief Picks an object
 * @param object the object
 * */
void pickObject(std::string object){

    //publish the pick in the topic
    if(!isDoubleObject_){
        std_msgs::String msg;
        msg.data = object;
        pick_pub_.publish(msg);
    }

    pickedObject_ = object;

    //execute the pick
    supervisor_msgs::Action action;
    action.actors.push_back(humanName_);
    action.name = "pick";
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(object);
    supervisor_msgs::HumanAction srv;
    srv.request.agent = humanName_;
    srv.request.action = action;
    if (!client_human_action_.call(srv)){
        ROS_ERROR("[human_simulator] Failed to call service human_monitor/human_action_simu");
    }


}

/**
 * \brief Callback for pick actions of the robot
 * @param msg topic msg
 * */
void pickCallback(const std_msgs::String::ConstPtr& msg){

    robotObject_ = msg->data;
}

/**
 * \brief Callback for asked actions by the robot
 * @param msg topic msg
 * */
void askCallback(const supervisor_msgs::Action::ConstPtr& msg){

    std_msgs::Bool ans;
    if(humanState_ == "ACTING"){
        //answer yes
        ans.data = true;
    }else if(humanState_ == "WAITING"){
        //answer no
        ans.data = false;
    }else{
        //decision needed
        ROS_WARN("[human_simulator] a decision is needed!");
    }
    ros::Duration(answerTime_).sleep();
    answer_pub_.publish(ans);
}

/**
 * \brief Callback for informed actions by the robot
 * @param msg topic msg
 * */
void informCallback(const supervisor_msgs::Action::ConstPtr& msg){

    if(msg->actors[0] == robotName_){
        if(!(humanState_ == "ACTING" && isDoubleObject_)){
            humanState_ = "WAITING";
        }
    }else if(msg->actors[0] == humanName_){
        if(humanState_ != "ACTING"){
            highLevelObject_ = objects_.front();
            currentObject_ = findObjectRefinment(humanName_, highLevelObject_);
            begin_ = ros::Time::now();
            humanState_ = "ACTING";
        }
    }else{
        ROS_WARN("[human_simulator] wrong action actor!");
    }
}



/**
 * \brief Callback of the previous action topic
 * @param msg topic msg
 * */
void previousCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    //we look if there was a change in the action performed (nothing is suppose to disappear from this list)
    std::vector<supervisor_msgs::Action> currentActions = msg->actions;
    if(currentActions.size() > oldMsgPrevious_.size()){
        //the action(s) performed are at the end of the list
        for(int i = oldMsgPrevious_.size(); i < currentActions.size(); i++){
            //we consider only robot actions
            if(currentActions[i].actors[0] == robotName_ && currentActions[i].succeed){
                robotObject_ = "NONE";
                supports_.erase(supports_.begin());
                objects_.erase(objects_.begin());
                if(!(humanState_ == "ACTING" && isDoubleObject_)){
                    humanState_ = "IDLE";
                }else{
                    isDoubleObject_ = false;
                }
            }
        }
    }
    oldMsgPrevious_ = currentActions;

}

/**
 * \brief Callback of the goal list
 * @param msg topic msg
 * */
void goalsListCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    if(!shouldStart_ && msg->currentGoal != "NONE"){
        shouldStart_ = true;
    }
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
  node_->getParam("human_simulator/objects", objects_);
  node_->getParam("human_simulator/supports", supports_);
  node_->getParam("human_simulator/humanName", humanName_);
  node_->getParam("human_simulator/pickTime", pickTime_);
  node_->getParam("human_simulator/placeTime", placeTime_);
  node_->getParam("human_simulator/answerTime", answerTime_);
  node_->getParam("supervisor/nbExp", nbExp_);

  int seed;
  node_->getParam("human_simulator/seed", seed);
  srand (seed);

  humanState_ = "IDLE";
  currentObject_ = "NONE";
  robotObject_ = "NONE";
  pickedObject_ = "NONE";
  shouldStart_ = false;
  nbConflict_ = 0;

  std::string filePath = "/home/sdevin/catkin_ws/src/supervisor/launchs/logs/hs_" + nbExp_ + ".dat";
  logFile_.open(filePath.c_str() , std::ios::out|std::ios::trunc);

  fillHighLevelNames();

  client_human_action_ = node_->serviceClient<supervisor_msgs::HumanAction>("human_monitor/human_action_simu");
  client_db_execute_ = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");

  ros::Subscriber sub_pick = node.subscribe("/action_executor/pick", 1, pickCallback);
  ros::Subscriber sub_previous = node_->subscribe("supervisor/previous_actions", 1, previousCallback);
  ros::Subscriber sub_ask = node_->subscribe("dialogue_node/asked_action", 1, askCallback);
  ros::Subscriber sub_inform = node_->subscribe("dialogue_node/inform_action", 1, informCallback);
  ros::Subscriber sub_goal = node.subscribe("/goal_manager/goalsList", 1, goalsListCallback);

  pick_pub_ = node.advertise<std_msgs::String>("human_simulator/pick", 1);
  answer_pub_ = node.advertise<std_msgs::Bool>("graphical_interface/boolAnswer", 1);

  ROS_INFO("[human_simulator] Ready");

  bool endTask = false;
  bool timerStarted = false;
  ros::Time start;
  while(node.ok() && !endTask){
      //activate the readers
      ros::spinOnce();
      if(!timerStarted & shouldStart_){
            timerStarted = true;
            start = ros::Time::now();
      }
      if(objects_.size() > 0){
            if(humanState_ == "IDLE" && shouldStart_){
                highLevelObject_ = objects_.front();
                ROS_INFO("[human_simulator] decision for %s", highLevelObject_.c_str());
                //check if there is more than one time the same object
                if(objects_.size() >1 && objects_[1] == highLevelObject_){
                    isDoubleObject_ = true;
                }else{
                    isDoubleObject_ = false;
                }
                //check if the human can access the object
                currentObject_ = findObjectRefinment(humanName_, highLevelObject_);
                ROS_INFO("[human_simulator] refinment: %s", currentObject_.c_str());
                std::string robotRefinment = findObjectRefinment(robotName_, highLevelObject_);
                if(currentObject_ != "NONE"){
                    //if there is two time the action to execute, the human does one
                    if(isDoubleObject_){
                        ROS_INFO("[human_simulator] acting: double object");
                        humanState_ = "ACTING";
                        begin_ = ros::Time::now();
                    }else if(robotObject_ == currentObject_){
                        //the robot is already performing the action
                        ROS_INFO("[human_simulator] waiting: robot doing!");
                        humanState_ = "WAITING";
                    }else if(robotRefinment != "NONE"){
                        //a decision is needed
                        if(mode_ == "all"){
                            //the human performs the action
                            ROS_INFO("[human_simulator] acting: with decision");
                            humanState_ = "ACTING";
                            begin_ = ros::Time::now();
                        }else if(mode_ == "none"){
                            //the human waits
                            ROS_INFO("[human_simulator] waiting: with decision");
                            humanState_ = "WAITING";
                        }else if(mode_ == "half"){
                            int nb = rand() % 10;
                            if(nb < 5){
                                //the human waits
                                ROS_INFO("[human_simulator] waiting: with decision");
                                humanState_ = "WAITING";
                            }else{
                                //the human performs the action
                                ROS_INFO("[human_simulator] acting: with decision");
                                humanState_ = "ACTING";
                                begin_ = ros::Time::now();
                            }
                        }else{
                            ROS_WARN("[human_simulator] Wrong mode: can be all, half or none");
                        }
                    }else{
                        ROS_INFO("[human_simulator] acting: only human");
                        //only the human can do it
                        humanState_ = "ACTING";
                        begin_ = ros::Time::now();
                    }
                }else{
                    //the human waits
                    humanState_ = "WAITING";
                }
            }else if(humanState_ == "ACTING"){
                if(pickedObject_ == "NONE"){
                    //pick action
                    ros::Time now = ros::Time::now();
                    ros::Duration d = now - begin_;
                    double duration = d.toSec();
                    if(duration >= pickTime_){
                        //check if the robot did not pick the object
                        if(!isDoubleObject_ && robotObject_ == currentObject_){
                            nbConflict_ ++;
                            humanState_ = "IDLE";
                        }else{
                            //we execute the pick action
                            pickObject(currentObject_);
                            begin_ = ros::Time::now();
                        }
                    }
                }else if(pickedObject_ == currentObject_){
                    //place action
                    ros::Time now = ros::Time::now();
                    ros::Duration d = now - begin_;
                    double duration = d.toSec();
                    if(duration >= placeTime_){
                        //we execute the place action
                        placeObject(currentObject_);
                    }
                }else{
                    ROS_WARN("[human_simulator] object to place: %s but current picked object is: %s!", pickedObject_.c_str(), currentObject_.c_str());
                }
            }
      }else{
          ROS_INFO("[human_simulator] End of the task!");
          endTask = true;
      }

      loop_rate.sleep();
  }

  ros::Time end = ros::Time::now();
  ros::Duration d = end - start;
  double tExp = d.toSec();
  logFile_ << "tExp: " << tExp << std::endl;
  logFile_ << "nbConflicts: " << nbConflict_ << std::endl;
  logFile_.close();
}

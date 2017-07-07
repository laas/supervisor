/**
author Sandra Devin

Main class of the goal_manager.

The goal manager allows to choose a goal to execute

**/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include "toaster_msgs/ExecuteDB.h"

#include "supervisor_msgs/Goal.h"
#include "supervisor_msgs/GoalsList.h"
#include "supervisor_msgs/String.h"

ros::NodeHandle* node_;
std::string robotName_;

std::string currentGoal_;
std::vector<std::string> previousGoals_;
std::vector<std::string> previousStatus_;
std::vector<std::string> pendingGoals_;
bool changed;

std::vector<std::string> goals_;
std::vector<std::string> status_;

std::vector<supervisor_msgs::Goal> possibleGoals_;

ros::ServiceClient* client_db_execute_;
ros::ServiceClient* client_say_;

ros::Time start_;
std::string nbParticipant_;
std::string condition_;
bool french;


/**
 * \brief Function which initialize the list of goals from the goal in param
 * */
void initGoals(){

    std::vector<std::string> names;
    node_->getParam("/goal_manager/goals/names", names);
    for(std::vector<std::string>::iterator it = names.begin(); it != names.end(); it++){
        //for each goal we retreive its actors and objective
        std::string actorsTopic = "/goal_manager/goals/" + *it + "_actors";
        std::vector<std::string> actors;
        node_->getParam(actorsTopic, actors);
        std::string objectiveTopic = "/goal_manager/goals/" + *it + "_objective";
        std::vector<std::string> objective;
        node_->getParam(objectiveTopic, objective);
        //we transform the objective into facts
        std::vector<toaster_msgs::Fact> obj;
        for(std::vector<std::string>::iterator ob = objective.begin(); ob != objective.end(); ob++){
            int beg = ob->find(',');
            int end = ob->find(',', beg+1);
            toaster_msgs::Fact fact;
            fact.subjectId = ob->substr(0, beg);
            fact.property = ob->substr(beg+1, end - beg - 1);
            fact.propertyType = "state";
            fact.targetId = ob->substr(end+1, ob->size() - end - 1);
            obj.push_back(fact);
        }
        //we then fill the goal list
        supervisor_msgs::Goal goal;
        goal.name = *it;
        goal.actors = actors;
        goal.objective = obj;
        possibleGoals_.push_back(goal);
    }

}

/**
 * \brief Service to add a new goal
 * @param req the name of the goal to add
 * @param res bool for success
 * @return true
 * */
bool newGoal(supervisor_msgs::String::Request  &req, supervisor_msgs::String::Response &res){

    //Check the data
    if(req.data == ""){
        ROS_WARN("[goal_manager] No goal specified!");
        res.success = false;
        return true;
    }

    //Chek if the goal is not already in progress
    if(currentGoal_ == req.data){
            ROS_WARN("[goal_manager] Goal already in progress!");
            res.success = false;
            return true;
    }

    //check if the goal is not already in the pending list
    for(std::vector<std::string>::iterator it = pendingGoals_.begin(); it != pendingGoals_.end(); it++){
        if(req.data == *it){
            ROS_WARN("[goal_manager] Goal already in the pending list!");
            res.success = false;
            return true;
        }
    }

    //check if the goal is a possible goal
    bool find = false;
    for(std::vector<supervisor_msgs::Goal>::iterator it = possibleGoals_.begin(); it != possibleGoals_.end(); it++){
        if(it->name == req.data){
            find = true;
            break;
        }
    }
    if(!find){
        ROS_WARN("[goal_manager] Goal %s not in the known goal!", currentGoal_.c_str());
        res.success = false;
        return true;
    }

    //add the goal to the pending list
    pendingGoals_.push_back(req.data);
    ROS_INFO("[goal_manager] Goal added to the pending list");

    changed = true;
    res.success = true;

    return true;
}

/**
 * \brief Service to cancel a goal
 * @param req the name of the goal to cancel
 * @param res bool for success
 * @return true
 * */
bool cancelGoal(supervisor_msgs::String::Request  &req, supervisor_msgs::String::Response &res){

    changed = true;
    //Check the data
    if(req.data == ""){
        ROS_WARN("[goal_manager] No goal specified!");
        res.success = false;
        return true;
    }

    //It is the current goal?
    if(currentGoal_ == req.data){
            ROS_INFO("[goal_manager] Stopping current goal");
            changed = true;
            currentGoal_ = "STOP";
            res.success = true;
            return true;
    }

    //look in the pending goals
    for(std::vector<std::string>::iterator it = pendingGoals_.begin(); it != pendingGoals_.end(); it++){
        if(req.data == *it){
            pendingGoals_.erase(it);
            ROS_INFO("[goal_manager] Goal cancelled");
            changed = true;
            res.success = true;
            return true;
        }
    }

    ROS_WARN("[goal_manager] No goal to cancel!");
    res.success = false;

    return true;
}

void writeLogs(){

   ros::Duration(1.0).sleep();

    std::ofstream fileSaveTime, fileSaveMistakes;
    //we save the execution time
    std::string fileNameTime = "/home/sdevin/catkin_ws/src/supervisor/logs/Time.txt";
    fileSaveTime.open(fileNameTime.c_str(), std::ios::app);
    ros::Time now = ros::Time::now();
    ros::Duration duration = now - start_;
    double d = duration.toSec();
    std::ostringstream strs;
    strs << d;
    std::string dString = strs.str();
    fileSaveTime << "Participant " << nbParticipant_.c_str() << "\t Condition " << condition_.c_str() << "\t " << dString.c_str() << std::endl;


    //we save the number of mistakes
    std::string fileNameMistakes = "/home/sdevin/catkin_ws/src/supervisor/logs/Mistakes.txt";
    fileSaveMistakes.open(fileNameMistakes.c_str(), std::ios::app);
    int nbMistakes = 0;

    //facts to check
    std::vector<toaster_msgs::Fact> facts;
    toaster_msgs::Fact fact;
    fact.property = "isScan";
    fact.targetId = "true";
    fact.subjectId = "BLUE_CUBE1";
    facts.push_back(fact);
    fact.subjectId = "BLUE_CUBE2";
    facts.push_back(fact);
    fact.subjectId = "BLUE_CUBE3";
    facts.push_back(fact);
    fact.subjectId = "GREEN_CUBE1";
    facts.push_back(fact);
    fact.subjectId = "GREEN_CUBE2";
    facts.push_back(fact);
    fact.subjectId = "GREEN_CUBE3";
    facts.push_back(fact);
    fact.subjectId = "RED_CUBE1";
    facts.push_back(fact);
    fact.subjectId = "RED_CUBE2";
    facts.push_back(fact);
    fact.subjectId = "RED_CUBE3";
    facts.push_back(fact);
    fact.property = "isIn";
    fact.targetId = "BLUE_BOX";
    fact.subjectId = "BLUE_CUBE1";
    facts.push_back(fact);
    fact.subjectId = "BLUE_CUBE2";
    facts.push_back(fact);
    fact.subjectId = "BLUE_CUBE3";
    facts.push_back(fact);
    fact.targetId = "GREEN_BOX";
    fact.subjectId = "GREEN_CUBE1";
    facts.push_back(fact);
    fact.subjectId = "GREEN_CUBE2";
    facts.push_back(fact);
    fact.subjectId = "GREEN_CUBE3";
    facts.push_back(fact);

    //we check the facts
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.type = "INDIV";
    srv.request.agent = robotName_;
    srv.request.facts = facts;
   int i = 0;
    if (client_db_execute_->call(srv)){
        std::vector<std::string> answer =  srv.response.results;
        for(std::vector<std::string>::iterator it = answer.begin(); it != answer.end(); it++){
            if(*it != "true"){
		ROS_WARN("mistake in first list, fact %d", i);
                nbMistakes++;
            }
	   i++;
        }
    }else{
       ROS_ERROR("[goal_manager] Failed to call service database_manager/execute");
    }

    //facts to check for red objects
    facts.clear();
    fact.property = "isScan";
    fact.targetId = "true";
    fact.subjectId = "RED_TAPE1";
    facts.push_back(fact);
    fact.subjectId = "RED_TAPE2";
    facts.push_back(fact);
    fact.property = "isIn";
    fact.targetId = "RED_BOX1";
    fact.subjectId = "RED_CUBE1";
    facts.push_back(fact);
    fact.targetId = "RED_BOX2";
    facts.push_back(fact);
    fact.targetId = "RED_BOX1";
    fact.subjectId = "RED_CUBE2";
    facts.push_back(fact);
    fact.targetId = "RED_BOX2";
    facts.push_back(fact);
    fact.targetId = "RED_BOX1";
    fact.subjectId = "RED_CUBE3";
    facts.push_back(fact);
    fact.targetId = "RED_BOX2";
    facts.push_back(fact);

    //we check the facts
    srv.request.facts = facts;
    i = 0;
    if (client_db_execute_->call(srv)){
        std::vector<std::string> answer =  srv.response.results;
        bool pair = true;
        bool ok = false;
        for(std::vector<std::string>::iterator it = answer.begin(); it != answer.end(); it++){
            if(*it != "true" && !pair && !ok){
                ROS_WARN("mistake in second list, fact %d", i);
                nbMistakes++;
            }else if(pair && *it == "true"){
                ok = true;
            }
            if(!pair){
                ok = false;
            }
            pair = !pair;
           i++;
        }
    }else{
       ROS_ERROR("[goal_manager] Failed to call service database_manager/execute");
    }


    facts.clear();
    fact.targetId = "RED_BOX1";
    fact.subjectId = "RED_TAPE1";
    facts.push_back(fact);
    fact.targetId = "RED_BOX2";
    facts.push_back(fact);
    fact.targetId = "RED_BOX1";
    fact.subjectId = "RED_TAPE2";
    facts.push_back(fact);
    fact.targetId = "RED_BOX2";
    facts.push_back(fact);

    //we check the facts
    srv.request.facts = facts;
    i = 0;
    if (client_db_execute_->call(srv)){
        std::vector<std::string> answer =  srv.response.results;
        bool ok = false;
        for(std::vector<std::string>::iterator it = answer.begin(); it != answer.end(); it++){
            if(*it == "true" ){
                ok = true;
		break;
            }
        }
	if(!ok){
		nbMistakes++;
	}
    }else{
       ROS_ERROR("[goal_manager] Failed to call service database_manager/execute");
    }

    std::ostringstream strs2;
    strs2 << nbMistakes;
    std::string mistakesString = strs2.str();
    fileSaveMistakes << "Participant " << nbParticipant_.c_str() << "\t Condition " << condition_.c_str() << "\t " << mistakesString.c_str() << std::endl;
}


/**
 * \brief Service call when a goal ends
 * @param req the name of the goal
 * @param res bool for success
 * @return true
 * */
bool endGoal(supervisor_msgs::String::Request  &req, supervisor_msgs::String::Response &res){

    //If we were in stopping process
    if(currentGoal_ == "STOP"){
        if(req.data == "OK"){
            //the goal has been stopped
            currentGoal_ = "NONE";
            ROS_INFO("[goal_manager] Goal stopped");
        }else{
            //else the name of the goal is returned
            currentGoal_ = req.data;
            ROS_INFO("[goal_manager] Failed to stop the current goal");
        }
        res.success = true;
        return true;

    }
    

    //Check if the goal is the current goal
/*    if(currentGoal_ != req.data){
        ROS_WARN("[goal_manager] %s is not the current goal!", req.data.c_str());;
        res.success = false;
        return true;
    }*/

    //Look for the goal objective
    /*std::vector<toaster_msgs::Fact> obj;
    for(std::vector<supervisor_msgs::Goal>::iterator it = possibleGoals_.begin(); it != possibleGoals_.end(); it++){
        if(it->name == currentGoal_){
            obj = it->objective;
            break;
        }
    }
    //Look if the objective of the goal is in the robot knowledge
    bool isIn;
    std::string status;
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.agent = robotName_;
    srv.request.facts = obj;
    if (client_db_execute_->call(srv)){
        isIn =  srv.response.boolAnswer;
    }else{
       ROS_ERROR("[goal_manager] Failed to call service database_manager/execute");
    }*/

    bool isIn = false;
    std::string status;
    if(req.data == "OK"){
        writeLogs();
        isIn = true;
    }

    //Publish the result
    if(isIn){
        ROS_INFO("[goal_manager] Goal succeed: %s", currentGoal_.c_str());
        status = "DONE";
        supervisor_msgs::String srv;
            if(french){
                srv.request.data = "Bravo!";
            }else{
                srv.request.data  = "Well done!";
            }
            if(!client_say_->call(srv)){
                ROS_WARN("[goal_manager] Unable to call service dialogue_node/Say");
            }

    }else{
        ROS_INFO("[goal_manager] Goal failed: %s", currentGoal_.c_str());
        status = "FAILED";
    }
    previousGoals_.push_back(currentGoal_);
    previousStatus_.push_back(status);
    currentGoal_ = "NONE";

    changed = true;
    res.success = true;

    return true;
}


/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "goal_manager");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[goal_manager] Init");

  initGoals();

  currentGoal_ = "NONE";
  changed = false;

  node_->getParam("/supervisor/robot/name", robotName_);
  node_->getParam("/supervisor/nbParticipant", nbParticipant_);
  node_->getParam("dialogue_node/french", french);

  std::string systemMode;
  node_->getParam("/supervisor/systemMode", systemMode);
  if(systemMode == "new"){
      std::string speakMode;
      node_->getParam("/robot_decision/mode", speakMode);
      if(speakMode == "negotiation"){
        condition_ = "Negotiation";
      }else{
        condition_ = "Adaptation";
      }
  }else{
      bool speakMode;
      node_->getParam("/supervisor/speakingMode", speakMode);
      if(speakMode){
        condition_ = "RS-all";
      }else{
        condition_ = "RS-none";
      }
  }

  //We create the publishers
  ros::Publisher goals_pub = node.advertise<supervisor_msgs::GoalsList>("goal_manager/goalsList", 1);

  ros::ServiceServer service_new = node.advertiseService("goal_manager/new_goal", newGoal); //add a new goal
  ros::ServiceServer service_cancel = node.advertiseService("goal_manager/cancel_goal", cancelGoal); //cancel a goal
  ros::ServiceServer service_end = node.advertiseService("goal_manager/end_goal", endGoal); //end a goal

  ros::ServiceClient client_db_execute = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
  client_db_execute_ = &client_db_execute;

  ros::ServiceClient client_say = node_->serviceClient<supervisor_msgs::String>("dialogue_node/say");
  client_say_ = &client_say;

  ROS_INFO("[goal_manager] Ready");

  while(node.ok()){
      //activate the readers
      ros::spinOnce();

      if(changed){
          //look if a new goal is needed
          if(currentGoal_ == "NONE" && pendingGoals_.size() > 0){
            //start a new goal
            currentGoal_ = pendingGoals_[0];
            pendingGoals_.erase(pendingGoals_.begin());
            ROS_INFO("[goal_manager] New goal: %s", currentGoal_.c_str());
            start_ = ros::Time::now();
	    supervisor_msgs::String srv;
	    if(currentGoal_ == "SCAN_US2"){
		currentGoal_ = "SCAN_US";
	    }else{
	    /*if(french){
	    	srv.request.data = "Commençons";
	    }else{
		srv.request.data = "Let's start!";
	    }
	    if(!client_say.call(srv)){
		ROS_WARN("[goal_manager] Unable to call service dialogue_node/Say");
	    } */
	    }
          }
          //compute the new list to publish
          goals_.clear();
          status_.clear();
          if(currentGoal_ != "NONE"){
              goals_.push_back(currentGoal_);
              status_.push_back("PROGRESS");
          }
          goals_.insert(goals_.end(), previousGoals_.begin(), previousGoals_.end());
          status_.insert(status_.end(), previousStatus_.begin(), previousStatus_.end());
          goals_.insert(goals_.end(), pendingGoals_.begin(), pendingGoals_.end());
          status_.insert(status_.end(), pendingGoals_.size(), "PENDING");
      }

      //publish the result
      supervisor_msgs::GoalsList msg;
      msg.goals = goals_;
      msg.status = status_;
      msg.changed = changed;
      msg.currentGoal = currentGoal_;
      goals_pub.publish(msg);

      changed = false;
      loop_rate.sleep();
  }
}

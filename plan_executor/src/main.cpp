/**
author Sandra Devin

Simple dialogue node

**/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

#include "supervisor_msgs/NewPlan.h"
#include "supervisor_msgs/ActionList.h"
#include "supervisor_msgs/HumanAction.h"
#include "supervisor_msgs/EndPlan.h"

using namespace std;

ros::NodeHandle* node;
string robotName_;
vector<supervisor_msgs::Action> robotActionsReady, robotActionsPlanned, agentXActionsReady, agentXActionsPlanned, humanActionsReady, humanActionsPlanned;
vector<supervisor_msgs::Link> links;
vector<int> actionsDone;
map<string, string> highLevelNames_;


/*
Fill the highLevelNames map from param
*/
void fillHighLevelNames(){

    //we retrieve the list objects from param and fill the map with their high level name
    vector<string> objects;
    node->getParam("/entities/objects", objects);
    for(vector<string>::iterator it = objects.begin(); it != objects.end(); it++){
        string topic = "/highLevelName/";
        topic = topic + *it;
        string highLevelName;
        if(node->hasParam(topic)){
            node->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for supports
    vector<string> support;
    node->getParam("/entities/supports", support);
    for(vector<string>::iterator it = support.begin(); it != support.end(); it++){
        string topic = "/highLevelName/";
        topic = topic + *it;
        string highLevelName;
        if(node->hasParam(topic)){
            node->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for containers
    vector<string> container;
    node->getParam("/entities/containers", container);
    for(vector<string>::iterator it = container.begin(); it != container.end(); it++){
        string topic = "/highLevelName/";
        topic = topic + *it;
        string highLevelName;
        if(node->hasParam(topic)){
            node->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
}

void evaluatePlan(){

    vector<supervisor_msgs::Action> newRobotActions, newAgentXActions, newHumanActions;

    for(vector<supervisor_msgs::Action>::iterator it = robotActionsPlanned.begin(); it != robotActionsPlanned.end(); it++){
        bool actionReady = true;
        for(vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                actionReady = false;
                for(vector<int>::iterator itd = actionsDone.begin(); itd != actionsDone.end(); itd++){
                    if(itl->origin == *itd){
                        actionReady = true;
                        break;
                    }
                }
                if(!actionReady){
                    break;
                }
            }
        }
        if(actionReady){
            robotActionsReady.push_back(*it);
        }else{
            newRobotActions.push_back(*it);
        }
    }
    for(vector<supervisor_msgs::Action>::iterator it = agentXActionsPlanned.begin(); it != agentXActionsPlanned.end(); it++){
        bool actionReady = true;
        for(vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                actionReady = false;
                for(vector<int>::iterator itd = actionsDone.begin(); itd != actionsDone.end(); itd++){
                    if(itl->origin == *itd){
                        actionReady = true;
                        break;
                    }
                }
                if(!actionReady){
                    break;
                }
            }
        }
        if(actionReady){
            agentXActionsReady.push_back(*it);
        }else{
            newAgentXActions.push_back(*it);
        }
    }
    for(vector<supervisor_msgs::Action>::iterator it = humanActionsPlanned.begin(); it != humanActionsPlanned.end(); it++){
        bool actionReady = true;
        for(vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                actionReady = false;
                for(vector<int>::iterator itd = actionsDone.begin(); itd != actionsDone.end(); itd++){
                    if(itl->origin == *itd){
                        actionReady = true;
                        break;
                    }
                }
                if(!actionReady){
                    break;
                }
            }
        }
        if(actionReady){
            humanActionsReady.push_back(*it);
        }else{
            newHumanActions.push_back(*it);
        }
    }

    robotActionsPlanned = newRobotActions;
    agentXActionsPlanned = newAgentXActions;
    humanActionsPlanned = newHumanActions;

}

/*
Service call when a plan is computed by the robot
*/
bool newPlan(supervisor_msgs::NewPlan::Request  &req, supervisor_msgs::NewPlan::Response &res){

    robotActionsReady.clear();
    robotActionsPlanned.clear();
    agentXActionsReady.clear();
    agentXActionsPlanned.clear();
    humanActionsReady.clear();
    humanActionsPlanned.clear();
    actionsDone.clear();
    links.clear();

    supervisor_msgs::Plan plan = req.plan;
    for(vector<supervisor_msgs::Action>::iterator it = plan.actions.begin(); it != plan.actions.end(); it++){
        if(it->actors[0] == robotName_){
            robotActionsPlanned.push_back(*it);
        }else if(it->actors[0] == "AGENTX" || it->actors[0] == "AGENTX2"){
            agentXActionsPlanned.push_back(*it);
        }else{
            humanActionsPlanned.push_back(*it);
        }
    }
    links = plan.links;
    evaluatePlan();

    return true;
}

/*
Service call when a plan is computed by the robot
*/
bool robotAction(supervisor_msgs::HumanAction::Request  &req, supervisor_msgs::HumanAction::Response &res){

    actionsDone.push_back(req.action.id);

    vector<supervisor_msgs::Action> newRobotActions;

    for(vector<supervisor_msgs::Action>::iterator it = robotActionsReady.begin(); it != robotActionsReady.end(); it++){
        if(!(req.action.id == it->id)){
            newRobotActions.push_back(*it);
        }
    }
    robotActionsReady = newRobotActions;
    evaluatePlan();

    return true;
}

pair<bool, int> isInList(supervisor_msgs::Action action, vector<supervisor_msgs::Action> actions){

    pair<bool, int> answer;
    for(vector<supervisor_msgs::Action>::iterator it = actions.begin(); it != actions.end(); it++){
        bool find = true;
        if(action.name != it->name){
            find = false;
        }else{
            if(action.parameters.size() != it->parameters.size()){
                find = false;
            }else{
                for(int i = 0; i < action.parameters.size(); i++){
                    if(highLevelNames_[action.parameters[i]] != highLevelNames_[it->parameters[i]]){
                        find = false;
                        break;
                    }
                }
            }
        }
        if(find){
            answer.first = true;
            answer.second = it->id;
            return answer;
        }
    }

    answer.first = false;
    answer.second = -1;
    return answer;
}

/*
Service call when a plan is computed by the robot
*/
bool humanAction(supervisor_msgs::HumanAction::Request  &req, supervisor_msgs::HumanAction::Response &res){
    //we look in the human actions
    pair<bool, int> humanAction = isInList(req.action, humanActionsReady);
    if(humanAction.first){
        actionsDone.push_back(humanAction.second);
        vector<supervisor_msgs::Action> newHumanActions;
        for(vector<supervisor_msgs::Action>::iterator it = humanActionsReady.begin(); it != humanActionsReady.end(); it++){
            if(!(humanAction.second == it->id)){
                newHumanActions.push_back(*it);
            }
        }
        humanActionsReady = newHumanActions;
        evaluatePlan();
    }else{
        //need replan
        ros::ServiceClient client = node->serviceClient<supervisor_msgs::EndPlan>("plan_elaboration/endPlan");
        supervisor_msgs::EndPlan srv;
        srv.request.report = false;
        if (!client.call(srv)){
            ROS_ERROR("[mental_state] Failed to call service plan_elaboration/endPlan");
        }
        robotActionsReady.clear();
        robotActionsPlanned.clear();
        agentXActionsReady.clear();
        agentXActionsPlanned.clear();
        humanActionsReady.clear();
        humanActionsPlanned.clear();
        actionsDone.clear();
        links.clear();
    }

    return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "plan_executor");
  ros::NodeHandle _node;
  node = &_node;
  ros::Rate loop_rate(30);
  node->getParam("/robot/name", robotName_);
  fillHighLevelNames();

  ros::Publisher robot_ready = node->advertise<supervisor_msgs::ActionList>("/plan_executor/actions_robot_ready", 1);
  ros::Publisher robot_planned = node->advertise<supervisor_msgs::ActionList>("/plan_executor/actions_robot_planned", 1);
  ros::Publisher x_ready = node->advertise<supervisor_msgs::ActionList>("/plan_executor/actions_x_ready", 1);
  ros::Publisher x_planned = node->advertise<supervisor_msgs::ActionList>("/plan_executor/actions_x_planned", 1);
  ros::Publisher human_ready = node->advertise<supervisor_msgs::ActionList>("/plan_executor/actions_human_ready", 1);
  ros::Publisher human_planned = node->advertise<supervisor_msgs::ActionList>("/plan_executor/actions_human_planned", 1);

  ros::ServiceServer service_new_plan = _node.advertiseService("plan_executor/newPlan", newPlan); //a new plan has been computed
  ros::ServiceServer service_robot_action = _node.advertiseService("plan_executor/robot_action", robotAction); //the robot did an action
  ros::ServiceServer service_human_action = _node.advertiseService("plan_executor/human_action", humanAction); //the human did an action

  ROS_INFO("[plan_executor] plan_executor ready");

  while (_node.ok()) {
      supervisor_msgs::ActionList msg;
      msg.actionsList = robotActionsReady;
      robot_ready.publish(msg);
      msg.actionsList = robotActionsPlanned;
      robot_planned.publish(msg);
      msg.actionsList = agentXActionsReady;
      x_ready.publish(msg);
      msg.actionsList = agentXActionsPlanned;
      x_planned.publish(msg);
      msg.actionsList = humanActionsReady;
      human_ready.publish(msg);
      msg.actionsList = humanActionsPlanned;
      human_planned.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

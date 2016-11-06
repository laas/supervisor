/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

#include "supervisor_msgs/NewPlan.h"
#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/HumanAction.h"


ros::NodeHandle* node;
std::string robotName_;
std::vector<supervisor_msgs::Action> robotActionsReady, robotActionsPlanned, agentXActionsReady, agentXActionsPlanned, humanActionsReady, humanActionsPlanned;
std::vector<supervisor_msgs::Link> links;
std::vector<int> actionsDone;


/**
 * \brief Update the plan
 * */
void evaluatePlan(){

    std::vector<supervisor_msgs::Action> newRobotActions, newAgentXActions, newHumanActions;

    for(std::vector<supervisor_msgs::Action>::iterator it = robotActionsPlanned.begin(); it != robotActionsPlanned.end(); it++){
        bool actionReady = true;
        for(std::vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                actionReady = false;
                for(std::vector<int>::iterator itd = actionsDone.begin(); itd != actionsDone.end(); itd++){
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
    for(std::vector<supervisor_msgs::Action>::iterator it = agentXActionsPlanned.begin(); it != agentXActionsPlanned.end(); it++){
        bool actionReady = true;
        for(std::vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                actionReady = false;
                for(std::vector<int>::iterator itd = actionsDone.begin(); itd != actionsDone.end(); itd++){
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
    for(std::vector<supervisor_msgs::Action>::iterator it = humanActionsPlanned.begin(); it != humanActionsPlanned.end(); it++){
        bool actionReady = true;
        for(std::vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                actionReady = false;
                for(std::vector<int>::iterator itd = actionsDone.begin(); itd != actionsDone.end(); itd++){
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

/**
 * \brief Service call when a plan is computed by the robot
 * @param req request of the service
 * @param res result of the service
 * @return true
 * */
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
    for(std::vector<supervisor_msgs::Action>::iterator it = plan.actions.begin(); it != plan.actions.end(); it++){
        if(it->actors[0] == robotName_){
            robotActionsPlanned.push_back(*it);
        }else if(it->actors[0] == "AGENTX"){
            agentXActionsPlanned.push_back(*it);
        }else{
            humanActionsPlanned.push_back(*it);
        }
    }
    links = plan.links;
    evaluatePlan();

    return true;
}

/**
 * \brief Evaluate if an actino is in a list
 * @param action the action tested
 * @param actions the list of actions
 * @return true if the action is in the list
 * */
std::pair<bool, int> isInList(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actions){

    std::pair<bool, int> answer;
    for(std::vector<supervisor_msgs::Action>::iterator it = actions.begin(); it != actions.end(); it++){
        bool find = true;
        if(action.name != it->name){
            find = false;
        }else{
            if(action.parameter_keys.size() != action.parameter_values.size()){
                ROS_WARN("[plan_executor] Invalid action parameters: nb keys should be equal to nb values!");
                find = false;
            }
            if(it->parameter_keys.size() != it->parameter_values.size()){
                ROS_WARN("[plan_executor] Invalid action parameters: nb keys should be equal to nb values!");
                find = false;
            }
            if(action.parameter_keys.size() != it->parameter_keys.size()){
                find = false;
            }
            for(int i = 0; i < action.parameter_keys.size(); i++){
                bool found = false;
                for(int j = 0; j < it->parameter_keys.size(); j++){
                    if(action.parameter_keys[i] == it->parameter_keys[j]){
                        if(action.parameter_values[i] != it->parameter_values[j]){
                            find = false;
                        }
                        found = true;
                        break;
                    }
                }
                if(!found){
                    find = false;
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

/**
 * \brief Service call when an action is executed by the robot
 * @param req request of the service
 * @param res result of the service
 * @return true
 * */
bool robotAction(supervisor_msgs::HumanAction::Request  &req, supervisor_msgs::HumanAction::Response &res){

    actionsDone.push_back(req.action.id);

    std::vector<supervisor_msgs::Action> newRobotActions;

    std::pair<bool, int> robotAction = isInList(req.action, robotActionsReady);
    if(robotAction.first){
        actionsDone.push_back(robotAction.second);
        for(std::vector<supervisor_msgs::Action>::iterator it = robotActionsReady.begin(); it != robotActionsReady.end(); it++){
            if(!(robotAction.second == it->id)){
                newRobotActions.push_back(*it);
            }
        }
    }
    robotActionsReady = newRobotActions;
    evaluatePlan();

    return true;
}

/**
 * \brief Service call when an action is executed by a human
 * @param req request of the service
 * @param res result of the service
 * @return true
 * */
bool humanAction(supervisor_msgs::HumanAction::Request  &req, supervisor_msgs::HumanAction::Response &res){
    //we look in the human actions
    std::pair<bool, int> humanAction = isInList(req.action, humanActionsReady);
    if(humanAction.first){
        actionsDone.push_back(humanAction.second);
        std::vector<supervisor_msgs::Action> newHumanActions;
        for(std::vector<supervisor_msgs::Action>::iterator it = humanActionsReady.begin(); it != humanActionsReady.end(); it++){
            if(!(humanAction.second == it->id)){
                newHumanActions.push_back(*it);
            }
        }
        humanActionsReady = newHumanActions;
        evaluatePlan();
    }else{
        //need replan
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

/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "plan_executor");
  ros::NodeHandle _node;
  node = &_node;
  ros::Rate loop_rate(30);
  node->getParam("/robot/name", robotName_);

  ros::ServiceServer service_new_plan = _node.advertiseService("plan_executor/newPlan", newPlan); //a new plan has been computed
  ros::ServiceServer service_robot_action = _node.advertiseService("plan_executor/robot_action", robotAction); //the robot did an action
  ros::ServiceServer service_human_action = _node.advertiseService("plan_executor/human_action", humanAction); //the human did an action

  ROS_INFO("[plan_executor] plan_executor ready");

  while (_node.ok()) {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

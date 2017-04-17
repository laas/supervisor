/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>

#include "supervisor_msgs/ActionsList.h"

ros::NodeHandle* node_;
std::vector<supervisor_msgs::Action> actionsTodo, previousActions;
std::vector<supervisor_msgs::Action> actionsTodoTopics, previousActionsTopics;
std::vector<supervisor_msgs::Action> actionsTodoPermanent, previousActionsPermanent;
std::vector<ros::Subscriber> todoSubs, previousSubs;
bool previousChanged, todoChanged;
int prevId_;

/**
 * \brief Compare 2 actions
 * @param action1 the fist action to compare
 * @param action2 the second action to compare
 * @return true if the actions are equal
 * */
bool areActionsEqual(supervisor_msgs::Action action1, supervisor_msgs::Action action2){

    //Compare names
    if(action1.name != action2.name){
        return false;
    }

    //Compare ids
    if(action1.id != action2.id){
        return false;
    }

    //Compare actors
    if(action1.actors.size() != action2.actors.size()){
        return false;
    }
    for(int i = 0; i < action1.actors.size(); i++){
        if(action1.actors[i] != action2.actors[i]){
            return false;
        }
    }

    //Compare parameters
    if(action1.parameter_keys.size() != action1.parameter_values.size()){
        ROS_WARN("[data_manager] Invalid action parameters: nb keys should be equal to nb values!");
        return false;
    }
    if(action2.parameter_keys.size() != action2.parameter_values.size()){
        ROS_WARN("[data_manager] Invalid action parameters: nb keys should be equal to nb values!");
        return false;
    }
    if(action1.parameter_keys.size() != action2.parameter_keys.size()){
        return false;
    }
    for(int i = 0; i < action1.parameter_keys.size(); i++){
        bool found = false;
        for(int j = 0; j < action2.parameter_keys.size(); j++){
            if(action1.parameter_keys[i] == action2.parameter_keys[j]){
                if(action1.parameter_values[i] != action2.parameter_values[j]){
                    return false;
                }
                found = true;
                break;
            }
        }
        if(!found){
            return false;
        }
    }

    //everything is the same, the actions are equal
    return true;
}

/**
 * \brief Callback for topics containing actions todo
 * @param msg topic msg
 * */
void todoCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    actionsTodoTopics.insert(actionsTodoTopics.end(), msg->actions.begin(), msg->actions.end());
    prevId_ = msg->prevId;
    if(msg->changed){
        todoChanged = true;
    }
}

/**
 * \brief Callback for topics containing previous actions
 * @param msg topic msg
 * */
void previousCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    previousActionsTopics.insert(previousActionsTopics.end(), msg->actions.begin(), msg->actions.end());
    if(msg->changed){
        previousChanged = true;
    }
}

/**
 * \brief Callback to add actions in actions todo list
 * @param msg topic msg
 * */
void addTodoCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    actionsTodoPermanent.insert(actionsTodoPermanent.end(), msg->actions.begin(), msg->actions.end());
    todoChanged = true;
}

/**
 * \brief Callback to remove actions in actions todo list
 * @param msg topic msg
 * */
void rmTodoCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> toRm = msg->actions;
    for(std::vector<supervisor_msgs::Action>::iterator it = toRm.begin(); it < toRm.end(); it++){
        for(std::vector<supervisor_msgs::Action>::iterator it2 = actionsTodoPermanent.begin(); it2 < actionsTodoPermanent.end(); it2++){
            if(areActionsEqual(*it, *it2)){
                actionsTodoPermanent.erase(it2);
                break;
            }
        }
    }
    todoChanged = true;
}

/**
 * \brief Callback to add actions in previous actions list
 * @param msg topic msg
 * */
void addPreviousCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    previousActionsPermanent.insert(previousActionsPermanent.end(), msg->actions.begin(), msg->actions.end());
    previousChanged = true;
}

/**
 * \brief Callback to remove actions in previous actions list
 * @param msg topic msg
 * */
void rmPreviousCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> toRm = msg->actions;
    for(std::vector<supervisor_msgs::Action>::iterator it = toRm.begin(); it < toRm.end(); it++){
        for(std::vector<supervisor_msgs::Action>::iterator it2 = previousActionsPermanent.begin(); it2 < previousActionsPermanent.end(); it2++){
            if(areActionsEqual(*it, *it2)){
                previousActionsPermanent.erase(it2);
                break;
            }
        }
    }
    previousChanged = true;
}

/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "data_manager");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[data_manager] Init");

  //first we get the topics lists
  std::vector<std::string> todoTopics, previousTopic;
  node.getParam("data_manager/actions_todo_topics", todoTopics);
  node.getParam("data_manager/previous_actions_topics", previousTopic);

  previousChanged = false;
  todoChanged = false;
  prevId_ = -1;

  //Then we create the readers
  ros::Subscriber sub;
  for(std::vector<std::string>::iterator it = todoTopics.begin(); it < todoTopics.end(); it++){
      sub = node.subscribe(*it, 1, todoCallback);
      todoSubs.push_back(sub);
  }
  for(std::vector<std::string>::iterator it = previousTopic.begin(); it < previousTopic.end(); it++){
      sub = node_->subscribe(*it, 1, previousCallback);
      previousSubs.push_back(sub);
  }
  ros::Subscriber sub_add_todo = node.subscribe("/data_manager/add_data/actions_todo", 1, addTodoCallback);
  ros::Subscriber sub_rm_todo = node.subscribe("/data_manager/rm_data/actions_todo", 1, rmTodoCallback);
  ros::Subscriber sub_add_previous = node.subscribe("/data_manager/add_data/previous_actions", 1, addPreviousCallback);
  ros::Subscriber sub_rm_previous = node.subscribe("/data_manager/rm_data/previous_actions", 1, rmPreviousCallback);

  //We create the publishers
  ros::Publisher todo_pub = node.advertise<supervisor_msgs::ActionsList>("supervisor/actions_todo", 1);
  ros::Publisher previous_pub = node.advertise<supervisor_msgs::ActionsList>("supervisor/previous_actions", 1);

  ROS_INFO("[data_manager] Ready");

  while(node.ok()){
      //activate the readers
      ros::spinOnce();

      //publish the result
      supervisor_msgs::ActionsList msg_todo;
      actionsTodo.insert(actionsTodo.end(), actionsTodoTopics.begin(), actionsTodoTopics.end());
      actionsTodo.insert(actionsTodo.end(), actionsTodoPermanent.begin(), actionsTodoPermanent.end());
      msg_todo.actions = actionsTodo;
      msg_todo.changed = todoChanged;
      msg_todo.prevId = prevId_;
      todo_pub.publish(msg_todo);
      supervisor_msgs::ActionsList msg_previous;
      previousActions.insert(previousActions.end(), previousActionsTopics.begin(), previousActionsTopics.end());
      previousActions.insert(previousActions.end(), previousActionsPermanent.begin(), previousActionsPermanent.end());
      msg_previous.actions = previousActions;
      msg_previous.changed = previousChanged;
      previous_pub.publish(msg_previous);


      //we clean lists
      previousChanged = false;
      todoChanged = false;
      actionsTodo.clear();
      previousActions.clear();
      actionsTodoTopics.clear();
      previousActionsTopics.clear();

      loop_rate.sleep();
  }
}

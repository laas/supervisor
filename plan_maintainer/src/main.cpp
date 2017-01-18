/**
author Sandra Devin

Main class of the plan maintainer

**/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

#include "std_srvs/Trigger.h"

#include "supervisor_msgs/SharedPlan.h"
#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/EndPlan.h"


ros::NodeHandle* node_;
std::string robotName_, xAgent_;
std::vector<supervisor_msgs::Action> oldMsgPrevious_, previousActions_, humansProgressActions_, todoActions_, plannedActions_;
supervisor_msgs::Action robotProgressAction_, plannedRobotAction_;
bool robotActing_;
std::vector<supervisor_msgs::Link> links;
std::map<std::string, std::string> highLevelNames_;
int currentPlan_;
ros::ServiceClient* client_stop_action_;
ros::ServiceClient* client_end_plan_;


/**
 * \brief Fill the highLevelNames map from param
 * */
void fillHighLevelNames(){

    //we retrieve the list objects from param and fill the map with their high level name
    std::vector<std::string> objects;
    node_->getParam("/entities/objects", objects);
    for(std::vector<std::string>::iterator it = objects.begin(); it != objects.end(); it++){
        std::string topic = "/highLevelName/" + *it;
        std::string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for supports
    std::vector<std::string> support;
    node_->getParam("/entities/supports", support);
    for(std::vector<std::string>::iterator it = support.begin(); it != support.end(); it++){
        std::string topic = "/highLevelName/" + *it;
        std::string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for containers
    std::vector<std::string> container;
    node_->getParam("/entities/containers", container);
    for(std::vector<std::string>::iterator it = container.begin(); it != container.end(); it++){
        std::string topic = "/highLevelName/" + *it;
        std::string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
}

/**
 * \brief Update the current plan
 * */
void updatePlan(){

    for(std::vector<supervisor_msgs::Action>::iterator it = plannedActions_.begin(); it != plannedActions_.end(); it++){
        bool linksOk = true;
        for(std::vector<supervisor_msgs::Link>::iterator itl = links.begin(); itl != links.end(); itl++){
            if(itl->following == it->id){
                linksOk = false;
                for(std::vector<supervisor_msgs::Action>::iterator itd = previousActions_.begin(); itd != previousActions_.end(); itd++){
                    if(itl->origin == itd->id){
                        linksOk = true;
                        break;
                    }
                }
                if(!linksOk){
                    break;
                }
            }
        }
        if(linksOk){
            /** @todo check if the preconditions are ok (how to manage the x agent?)*/
            todoActions_.push_back(*it);
        }

    }

    /** @todo check if the todo actions are still feasible*/
    /** @todo check if no more todo/in progress actions in the plan*/

}


/**
 * \brief Say if an action is in a list (considering high level objects)
 * @param action the looking action
 * @param actions the list
 * @return the corresponding action of the list (action with id = -1 if the action is not in the list)
 * */
supervisor_msgs::Action isInList(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actions){

    for(std::vector<supervisor_msgs::Action>::iterator it = actions.begin(); it != actions.end(); it++){
        bool find = true;
        if(action.name != it->name){
            find = false;
        }else{
            if(action.actors.size() != it->actors.size()){
                find = false;
            }else{
                for(int i = 0; i < action.actors.size(); i++){
                    if(action.actors[i] != it->actors[i]){
                        find = false;
                        break;
                    }
                }
                if(find){
                    if(action.parameter_values.size() != it->parameter_values.size()){
                        find = false;
                    }else{
                        for(int i = 0; i < action.parameter_values.size(); i++){
                            if(highLevelNames_[action.parameter_values[i]] != highLevelNames_[it->parameter_values[i]]){
                                find = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
        if(find){
            return *it;
        }
    }

    supervisor_msgs::Action toReturn;
    toReturn.id = -1;
    return toReturn;
}

/**
 * \brief Clear all data about the current plan
 * */
void endCurrentPlan(){

    currentPlan_ = -1;
    previousActions_.clear();
    todoActions_.clear();
    plannedActions_.clear();
}


/**
 * \brief Function which return te object to lock given an action
 * @param action the attributed action
 * @return the object to lock
 * */
std::string getLockedObject(supervisor_msgs::Action action){

    if(action.name == "pick" || action.name == "pickandplace" || action.name == "pickandplacereachable" || action.name == "pickanddrop"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                return action.parameter_values[i];
            }
        }
    }else if(action.name == "place"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "support"){
                return action.parameter_values[i];
            }
        }
    }else if(action.name == "drop"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "container"){
                return action.parameter_values[i];
            }
        }
    }

    return action.parameter_values[0];
}

/**
 * \brief Function wich check if an action is from the current plan and react accordingly
 * @param action the action to check
 * @param the actor of the action
 * */
void checkAction(supervisor_msgs::Action action, std::string actor){

    //look if the action was in the plan and todo
    supervisor_msgs::Action plannedAction = isInList(action, todoActions_);
    if(plannedAction.id == -1){
        //it is a unexpected action, a new plan is needed
        supervisor_msgs::EndPlan srv;
        srv.request.success = false;
        srv.request.evaluate = false;
        if (!client_end_plan_->call(srv)){
           ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
        }
        endCurrentPlan();
    }else{
        //look if the agent is the actor of the planned action
        if(plannedAction.actors[0] == actor){
            //the action was planned
            if(actor == robotName_){
                plannedRobotAction_ = plannedAction;
            }else{
                //for humans, we do not consider for now actions in progress, only previous actions
                previousActions_.push_back(plannedAction);
            }
        }else if(plannedAction.actors[0] == xAgent_){
            //the plan need to be re-evaluate
            supervisor_msgs::EndPlan srv;
            srv.request.success = false;
            srv.request.evaluate = true;
            srv.request.objectLocked = getLockedObject(action);
            srv.request.agentLocked = actor;
            if (!client_end_plan_->call(srv)){
               ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
            }
            endCurrentPlan();
        }else{
            //it is a unexpected action, a new plan is needed
            supervisor_msgs::EndPlan srv;
            srv.request.success = false;
            srv.request.evaluate = false;
            if (!client_end_plan_->call(srv)){
               ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
            }
            endCurrentPlan();
        }
    }
}

/**
 * \brief Function wich check the current actions when a new plan arrives
 * */
void checkCurrentActions(){

    if(!robotActing_){
        checkAction(robotProgressAction_, robotName_);
    }
}

/**
 * \brief Callback of the plan topic
 * @param msg topic msg
 * */
void planCallback(const supervisor_msgs::SharedPlan::ConstPtr& msg){

    if(currentPlan_ == -1){
        //a new plan arrived!
        currentPlan_ = msg->id;
        plannedActions_ = msg->actions;
        links = msg->links;
        updatePlan();
        checkCurrentActions();
    }else if(currentPlan_ != msg->id){
        //the plan changed
        endCurrentPlan();
        plannedActions_= msg->actions;
        links = msg->links;
        updatePlan();
    }


}


/**
 * \brief Callback of the robot action topic
 * @param msg topic msg
 * */
void robotActionCallback(const supervisor_msgs::Action::ConstPtr& msg){

    if(!robotActing_){
        //its a new action!
        robotActing_ = true;
        robotProgressAction_ = *msg;
        checkAction(robotProgressAction_, robotName_);
    }

}


/**
 * \brief Callback of the previous action topic
 * @param msg topic msg
 * */
void previousActionCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    //we look if there was a change in the action performed (nothing is suppose to disappear from this list)
    std::vector<supervisor_msgs::Action> currentActions = msg->actions;
    if(currentActions.size() > oldMsgPrevious_.size()){
        //the action(s) performed are at the end of the list
        for(int i = oldMsgPrevious_.size(); i < currentActions.size(); i++){
            if(robotActing_ && robotProgressAction_.id == currentActions[i].id){
                //it was the robot current action
                robotActing_ = false;
                if(!currentActions[i].succeed){
                    //the action failed, a new plan is needed
                    supervisor_msgs::EndPlan srv;
                    srv.request.success = false;
                    srv.request.evaluate = false;
                    if (!client_end_plan_->call(srv)){
                       ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                    }
                    endCurrentPlan();
                }else{
                    //the action succeed, we update the plan
                    previousActions_.push_back(plannedRobotAction_);
                    updatePlan();
                }
            }else{
                //it is not the robot action
                checkAction(currentActions[i], currentActions[i].actors[0]);
                if(currentPlan_ != -1){
                    //the plan was not broked by this action
                    updatePlan();
                }
            }
        }
    }
    oldMsgPrevious_ = currentActions;

}

/**
 * \brief Service call when a plan needs to be stopped
 * @param req the request of the service
 * @param res answer of the service
 * @return true
 * */
bool stopSrv(std_srvs::Trigger ::Request  &req, std_srvs::Trigger ::Response &res){

    endCurrentPlan();
    res.success = true;
    if(robotActing_){
        //transmit the order to the robot decision
        std_srvs::Trigger srv;
        if (client_stop_action_->call(srv)){
            res.success = srv.response.success;
        }else{
            res.success = false;
            ROS_ERROR("[plan_elaboration] Failed to call service robot_decision/stop");
        }
    }

    return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "plan_maintainer");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate loop_rate(30);
  node_->getParam("/robot/name", robotName_);
  node_->getParam("/supervisor/AgentX", xAgent_);
  fillHighLevelNames();
  currentPlan_ = -1;
  robotActing_ = false;

  ros::Subscriber sub_plan = node_->subscribe("plan_elaboration/plan", 1, planCallback);
  ros::Subscriber sub_robot_action = node_->subscribe("/action_executor/current_robot_action", 1, robotActionCallback);
  ros::Subscriber sub_humans_action = node_->subscribe("supervisor/previous_actions", 1, previousActionCallback);

  ros::ServiceServer service_end = node.advertiseService("plan_maintainer/stop", stopSrv); //when a plan needs to be stopped

  ros::ServiceClient client_end_plan = node_->serviceClient<supervisor_msgs::EndPlan>("plan_elaboration/end_plan");
  client_end_plan_ = &client_end_plan;
  ros::ServiceClient client_stop_action = node_->serviceClient<std_srvs::Trigger>("robot_decision/stop");
  client_stop_action_ = &client_stop_action;


  ros::Publisher todo_actions = node_->advertise<supervisor_msgs::ActionsList>("/plan_maintainer/todo_actions", 1);
  ros::Publisher planned_actions = node_->advertise<supervisor_msgs::ActionsList>("/plan_maintainer/planned_actions", 1);
  ROS_INFO("[plan_maintainer] plan_maintainer ready");

  while (node.ok()) {
      ros::spinOnce();
      supervisor_msgs::ActionsList msg;
      msg.actions = todoActions_;
      planned_actions.publish(msg);
      msg.actions = plannedActions_;
      planned_actions.publish(msg);
      loop_rate.sleep();
  }

  return 0;
}

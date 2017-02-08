/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "std_srvs/Trigger.h"

#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/Ask.h"
#include "supervisor_msgs/EndPlan.h"

typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;

ros::NodeHandle* node_;
ros::ServiceClient* client_end_plan_;
ros::ServiceClient* client_ask_;
std::string robotName_, robotState_, xAgent_, mainPartner_;
Client* actionClient_;
supervisor_msgs::Action currentAction_;
std::map<std::string, std::string> highLevelNames_;
std::map<std::string, std::vector<std::string> > highLevelRefinment_;
std::string mode_;
bool timerStarted_;
std::clock_t start_;
double timeAdaptation_, timeWaitHuman_;
int previousManagedAction_;


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
    }else if(action.name == "place" || action.name == "placereachable"){
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
 * \brief Says if an action as a "twin" in the todo list
 * @param action the action we are testing
 * @param actionsTodo list of the todo actions
 * @return true if the action is an identical action
 * */
bool isIdendicalAction(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actionsTodo){

    for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo.begin(); it != actionsTodo.end(); it++){
        if(action.id == it->id){
            continue;
        }
        bool find = true;
        if(action.name != it->name){
            find = false;
        }else if(it->actors[0] != xAgent_){
            //this process applies only for agent x actions
            find = false;
        }else{
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
        if(find){
            return true;
        }
    }

    return false;

}

/**
 * \brief Callback for the end of the robot action
 * @param state the state of the action server
 * @param result the result of the action server
 * */
void actionDone(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result){

    robotState_ = "IDLE";
}



/**
 * \brief Callback for todo actions
 * @param msg topic msg
 * */
void todoCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> actionsTodo = msg->actions;

    if(robotState_ == "IDLE" && actionsTodo.size() > 0){
        bool isWaiting = false;

        supervisor_msgs::Action action;
        bool hasXAction = false;
        bool hasRobotAction = false;
        //we look for actions with the robot has actor
        for(std::vector<supervisor_msgs::Action>::iterator it = actionsTodo.begin(); it != actionsTodo.end(); it++){
            if(it->actors[0] == robotName_){
                hasRobotAction = true;
                action = *it;
                break;
            }else if(!hasXAction && it->actors[0] == xAgent_){
                //we remember the action in case we do not find a robot action
                hasXAction = true;
                action = *it;
            }
        }
        if(action.id == previousManagedAction_ && !timerStarted_){
            //we wait for the todo list update
            return;
        }
        previousManagedAction_ = action.id;
        if(hasRobotAction){
            //we execute the action
            supervisor_msgs::ActionExecutorGoal goal;
            action.actors[0] = robotName_;
            goal.action = action;
            actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
            robotState_ = "ACTING";
            currentAction_ = action;
        }else if(hasXAction){
            //we try to attribute the action
            /** @todo implement priority for identical actions*/
            if(isIdendicalAction(action, actionsTodo)){
                //if there is more than once this action to attribute we execute the action
                supervisor_msgs::ActionExecutorGoal goal;
                action.actors[0] = robotName_;
                goal.action = action;
                actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                robotState_ = "ACTING";
                currentAction_ = action;

                supervisor_msgs::EndPlan srv;
                srv.request.success = false;
                srv.request.evaluate = true;
                srv.request.objectLocked = getLockedObject(action);
                srv.request.agentLocked = robotName_;
                if (!client_end_plan_->call(srv)){
                   ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                }
            }else{
                //we try to attribute the action
                /** @todo check the possible actors*/
                std::vector<std::string> possibleActors;
                if(possibleActors.size() == 1){
                    supervisor_msgs::EndPlan srv;
                    srv.request.success = false;
                    srv.request.evaluate = true;
                    srv.request.objectLocked = getLockedObject(action);
                    srv.request.agentLocked = possibleActors[0];
                    if (!client_end_plan_->call(srv)){
                       ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                    }
                    if(possibleActors[0] == robotName_){
                        //and we execute the action
                        supervisor_msgs::ActionExecutorGoal goal;
                        action.actors[0] = robotName_;
                        goal.action = action;
                        actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                        robotState_ = "ACTING";
                        currentAction_ = action;

                        supervisor_msgs::EndPlan srv;
                        srv.request.success = false;
                        srv.request.evaluate = true;
                        srv.request.objectLocked = getLockedObject(action);
                        srv.request.agentLocked = robotName_;
                        if (!client_end_plan_->call(srv)){
                           ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                        }
                    }
                //}else if(possibleActors.size() > 1){
                }else{
                    if(mode_ == "negotiation"){
                        supervisor_msgs::Ask srv_ask;
                        srv_ask.request.type = "ACTION";
                        srv_ask.request.subType = "WANT";
                        srv_ask.request.action = action;
                        srv_ask.request.receiver = mainPartner_;
                        srv_ask.request.waitForAnswer = true;
                        if (client_ask_->call(srv_ask)){
                          if(srv_ask.response.boolAnswer){
                              //the partner wants to perform the action
                              supervisor_msgs::EndPlan srv;
                              srv.request.success = false;
                              srv.request.evaluate = true;
                              srv.request.objectLocked = getLockedObject(action);
                              srv.request.agentLocked = mainPartner_;
                              if (!client_end_plan_->call(srv)){
                                 ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                              }
                          }else{
                              //the partner does not want to perform the action: the robot does it
                              supervisor_msgs::ActionExecutorGoal goal;
                              action.actors[0] = robotName_;
                              goal.action = action;
                              actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                              robotState_ = "ACTING";
                              currentAction_ = action;

                              supervisor_msgs::EndPlan srv;
                              srv.request.success = false;
                              srv.request.evaluate = true;
                              srv.request.objectLocked = getLockedObject(action);
                              srv.request.agentLocked = robotName_;
                              if (!client_end_plan_->call(srv)){
                                 ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                              }
                          }
                        }else{
                          ROS_ERROR("Failed to call service dialogue_node/ask");
                        }
                    }else if(mode_ == "adaptation"){
                        //we wait few time to see if the partner performs the action, if not the robot does it
                        isWaiting = true;
                        if(!timerStarted_){
                            start_ = clock();
                            timerStarted_ = true;
                        }else{
                            double duration = (clock() - start_ ) / (double) CLOCKS_PER_SEC;
                            if(duration >= timeAdaptation_){
                                //the partner does not perform it, the robot does it
                                supervisor_msgs::ActionExecutorGoal goal;
                                action.actors[0] = robotName_;
                                goal.action = action;
                                actionClient_->sendGoal(goal,  &actionDone, Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
                                robotState_ = "ACTING";
                                currentAction_ = action;

                                supervisor_msgs::EndPlan srv;
                                srv.request.success = false;
                                srv.request.evaluate = true;
                                srv.request.objectLocked = getLockedObject(action);
                                srv.request.agentLocked = robotName_;
                                if (!client_end_plan_->call(srv)){
                                   ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                                }
                                timerStarted_ = false;
                            }
                        }
                    }else{
                        ROS_WARN("[robot_decision] Actual mode is: %s, it should be negotiation or adaptation", mode_.c_str());
                    }
                }
            }
        }else{
            //there is only actions to do for the human
            isWaiting = true;
            if(!timerStarted_){
                start_ = clock();
                timerStarted_ = true;
            }else{
                double duration = (clock() - start_ ) / (double) CLOCKS_PER_SEC;
                if(duration >= timeWaitHuman_){
                    //the partner does not perform its action, we look for another plan
                    timerStarted_ = false;
                    supervisor_msgs::EndPlan srv;
                    srv.request.success = false;
                    srv.request.evaluate = false;
                    srv.request.forgiveAction = true;
                    srv.request.objectLocked = getLockedObject(actionsTodo[0]);
                    srv.request.agentLocked = mainPartner_;
                    if (!client_end_plan_->call(srv)){
                       ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                    }
                }
            }
        }

        if(!isWaiting){
            timerStarted_ = false;
        }
    }


}

/**
 * \brief Service call when the current robot action needs to be stopped
 * @param req the request of the service
 * @param res answer of the service
 * @return true
 * */
bool stopSrv(std_srvs::Trigger ::Request  &req, std_srvs::Trigger ::Response &res){

    if(robotState_ == "IDLE"){
        //stop the action
        actionClient_->cancelGoal();
        robotState_ = "STOPING";
        while(robotState_ == "STOPING"){
            ros::spinOnce();
        }
        res.success = true;
    }else{
        res.success = true;
    }

    return true;
}


/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "robot_decision");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);
  node_ = &node;

  ROS_INFO("[robot_decision] Init");

  node.getParam("supervisor/robot/name", robotName_);
  node.getParam("supervisor/AgentX", xAgent_);
  node.getParam("robot_decision/mode", mode_);
  node_->getParam("/supervisor/mainPartner", mainPartner_);
  node_->getParam("/robot_decision/timeAdaptation", timeAdaptation_);
  node_->getParam("/robot_decision/timeWaitHuman", timeWaitHuman_);

  robotState_ = "IDLE";
  timerStarted_ = false;
  previousManagedAction_ = -1;

  fillHighLevelNames();

  ros::ServiceClient client_ask = node_->serviceClient<supervisor_msgs::Ask>("dialogue_node/ask");
  client_ask_ = &client_ask;
  ros::ServiceClient client_end_plan = node_->serviceClient<supervisor_msgs::EndPlan>("plan_elaboration/end_plan");
  client_end_plan_ = &client_end_plan;


  ROS_INFO("[robot_decision] Waiting for action executor server");
  Client actionClient("supervisor/action_executor", true);
  actionClient.waitForServer();
  actionClient_ = &actionClient;

  ros::Subscriber sub_todo = node.subscribe("supervisor/actions_todo", 1,todoCallback);

  ros::ServiceServer service_stop = node.advertiseService("robot_decision/stop", stopSrv); //when an action needs to be stopped

  ROS_INFO("[robot_decision] Ready");

  while(node.ok()){
      //activate the readers
      ros::spinOnce();

      /** @todo check mental states*/
      /** @todo add management of arms retract*/
      loop_rate.sleep();
  }
}

/**
author Sandra Devin

Main class of the plan maintainer

**/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include "boost/algorithm/string.hpp"

#include "std_srvs/Trigger.h"

#include "supervisor_msgs/SharedPlan.h"
#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/GoalsList.h"
#include "toaster_msgs/ExecuteDB.h"



ros::NodeHandle* node_;
std::string robotName_, xAgent_;
std::vector<supervisor_msgs::Action> oldMsgPrevious_, previousActions_, humansProgressActions_, todoActions_, plannedActions_;
supervisor_msgs::Action robotProgressAction_, plannedRobotAction_, toTreatRobotResult_;
bool robotActing_;
std::vector<supervisor_msgs::Link> links;
std::map<std::string, std::string> highLevelNames_;
std::map<std::string, std::vector<std::string> > highLevelRefinment_;
int currentPlan_;
ros::ServiceClient* client_stop_action_;
ros::ServiceClient* client_end_plan_;
int lastPlanId_;
bool changed_;
int prevId_;
bool actionRobotFinished = false;


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
 * \brief Clear all data about the current plan
 * */
void endCurrentPlan(){

    currentPlan_ = -1;
    changed_ = true;
    previousActions_.clear();
    todoActions_.clear();
    plannedActions_.clear();
}



/**
 * \brief Update the current plan
 * */
void updatePlan(){

    std::vector<supervisor_msgs::Action> newPlanned, newTodo, toCheck;
    toCheck.insert(toCheck.begin(),plannedActions_.begin(),plannedActions_.end());
    toCheck.insert(toCheck.begin(),todoActions_.begin(),todoActions_.end());
    for(std::vector<supervisor_msgs::Action>::iterator it = toCheck.begin(); it != toCheck.end(); it++){
        bool executed = false;
        if(it->id == plannedRobotAction_.id && actionRobotFinished){
            //the action is over
	     	actionRobotFinished = false;
            executed = true;
            continue;
        }else{
            for(std::vector<supervisor_msgs::Action>::iterator itd = previousActions_.begin(); itd != previousActions_.end(); itd++){
                if(it->id == itd->id){
                    //the action has been executed
                    executed = true;
                    break;
                }
            }
        }
        if(executed){
            continue;
        }
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
            newTodo.push_back(*it);
        }else{
            newPlanned.push_back(*it);
        }

    }
    changed_ = true;
    plannedActions_ = newPlanned;
    todoActions_ = newTodo;

    /** @todo check if the todo actions are still feasible*/
   if(plannedActions_.size() == 0 && todoActions_.size() == 0 && !robotActing_){
	//end of the plan
	supervisor_msgs::EndPlan srv;
    srv.request.success = true;
    srv.request.evaluate = false;
    if (!client_end_plan_->call(srv)){
       ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
    }
    endCurrentPlan();
   }
}


/**
 * \brief Say if an action is in a list (considering high level objects)
 * @param action the looking action
 * @param actions the list
 * @return the corresponding action of the list (action with id = -1 if the action is not in the list)
 * */
supervisor_msgs::Action isInList(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actions){

    bool hasAgentXAction = false;
    bool hasAction = false;
    supervisor_msgs::Action toReturn;
    for(std::vector<supervisor_msgs::Action>::iterator it = actions.begin(); it != actions.end(); it++){
        bool find = true;
        if(action.name == it->name || (action.name == "pick" && boost::contains(it->name, "pick"))){
            if(action.name == "pick" && it->name != "pick"){
                //we only check the object
                std::string object;
                for(int i = 0; i < action.parameter_keys.size(); i++){
                    if(action.parameter_keys[i] == "object"){
                        object = action.parameter_values[i];
                        break;
                    }
                }
                for(int i = 0; i < it->parameter_keys.size(); i++){
                    if(it->parameter_keys[i] == "object"){
                        if(highLevelNames_[it->parameter_values[i]] != highLevelNames_[object]){
                            find = false;
                        }
                    }
                }
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
        }else{
            find = false;
        }
        if(find){
            //we keep in priority actions from the actor, then agentX actions, then others
            if(action.actors[0] == it->actors[0]){
                return *it;
            }else if(!hasAgentXAction && it->actors[0] == xAgent_){
                hasAgentXAction = true;
                hasAction = true;
                toReturn = *it;
            }else if(!hasAction){
                hasAction = true;
                toReturn = *it;
            }
        }
    }

    if(!hasAction){
        toReturn.id = -1;
    }

    return toReturn;
}

/**
 * \brief Function which return te object to lock given an action
 * @param action the attributed action
 * @return the object to lock
 * */
std::string getLockedObject(supervisor_msgs::Action action){

    std::string object;

    if(action.name == "pick" || action.name == "pickandplace" || action.name == "pickandplacereachable" || action.name == "pickanddrop"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                object = action.parameter_values[i];
            }
        }
    }else if(action.name == "place"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "support"){
                object = action.parameter_values[i];
            }
        }
    }else if(action.name == "drop"){
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "container"){
                object = action.parameter_values[i];
            }
        }
    }

    if(highLevelRefinment_[object].size() > 0){
		//need to refine the object
		std::vector<toaster_msgs::Fact> toTest;
		for(std::vector<std::string>::iterator it = highLevelRefinment_[object].begin(); it != highLevelRefinment_[object].end(); it++){
			toaster_msgs::Fact fact;
			fact.subjectId = *it;
			fact.property = "isReachableBy";
			fact.targetId = action.actors[0];
			toTest.push_back(fact);
		}
		std::vector<std::string> res;
			toaster_msgs::ExecuteDB srv;
			srv.request.command = "ARE_IN_TABLE";
			srv.request.type = "INDIV";
			srv.request.agent = robotName_;
			srv.request.facts = toTest;
		ros::ServiceClient client_db_execute_ = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
			if (client_db_execute_.call(srv)){
		      std::vector<std::string> res = srv.response.results;
		  for(int i = 0; i < res.size(); i++){
			if(res[i] == "true"){
				return highLevelRefinment_[object][i];
			}
		  }
		}else{
	   	 ROS_ERROR("[action_executor] Failed to call service database_manager/execute");
		}
    }

    return object;
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
                if(action.name == "pick" && plannedAction.name != "pick"){
                    //if the action was a pick, we replace the todo action by a place
                    for(std::vector<supervisor_msgs::Action>::iterator it = todoActions_.begin(); it != todoActions_.end(); it++){
                        if(it->id == plannedAction.id){
                            //we remove the pickand from the name
                            std::string newName = it->name.substr(7);
                            it->name = newName;
                        }
                    }
                }else{
                    previousActions_.push_back(plannedAction);
                }
            }
        }else if(actor != robotName_ && plannedAction.actors[0] == xAgent_){
            if(action.name == "pick" && plannedAction.name != "pick"){
                //if the action was a pick, we replace the todo action by a place
                for(std::vector<supervisor_msgs::Action>::iterator it = todoActions_.begin(); it != todoActions_.end(); it++){
                    if(it->id == plannedAction.id){
                        //we remove the pickand from the name
                        std::string newName = it->name.substr(7);
                        it->name = newName;
                    }
                }
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
            }
        }else if(actor != robotName_){
            //it is a unexpected action, a new plan is needed
            supervisor_msgs::EndPlan srv;
            srv.request.success = false;
            srv.request.evaluate = false;
            if (!client_end_plan_->call(srv)){
               ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
            }
            endCurrentPlan();
            //if the action was the robot one, we stop the robot action
            if(plannedAction.id == plannedRobotAction_.id){
                std_srvs::Trigger srv;
                if (!client_stop_action_->call(srv)){
                    ROS_ERROR("[plan_elaboration] Failed to call service robot_decision/stop");
                }
            }
        }else{
            endCurrentPlan();
        }
    }
}

/**
 * \brief Function wich check the current actions when a new plan arrives
 * */
void checkCurrentActions(){

    if(robotActing_){
        checkAction(robotProgressAction_, robotName_);
        updatePlan();
    }
}

/**
 * \brief Callback of the plan topic
 * @param msg topic msg
 * */
void planCallback(const supervisor_msgs::SharedPlan::ConstPtr& msg){

    if(currentPlan_ == -1 && msg->id > lastPlanId_){
        //a new plan arrived!
        currentPlan_ = msg->id;
        lastPlanId_ = msg->id;
        plannedActions_ = msg->actions;
        links = msg->links;
        updatePlan();
        checkCurrentActions();
    }else if(currentPlan_ != -1 && msg->id > currentPlan_){
        //the plan changed
        endCurrentPlan();
        currentPlan_ = msg->id;
        plannedActions_= msg->actions;
        lastPlanId_ = msg->id;
        links = msg->links;
        updatePlan();
        checkCurrentActions();
    }


}


/**
 * \brief Callback of the robot action topic
 * @param msg topic msg
 * */
void robotActionCallback(const supervisor_msgs::Action::ConstPtr& msg){

    if(!robotActing_ && msg->name != "" && toTreatRobotResult_.id == msg->id){
        //the action is already over, we treat the result
        if(!toTreatRobotResult_.succeed && currentPlan_ != -1){
            //the action failed, a new plan is needed
            supervisor_msgs::EndPlan srv;
            srv.request.success = false;
            srv.request.evaluate = false;
            if (!client_end_plan_->call(srv)){
               ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
            }
            endCurrentPlan();
        }else if(currentPlan_ != -1){
            //the action succeed, we update the plan
            checkAction(robotProgressAction_, robotName_);
            previousActions_.push_back(plannedRobotAction_);
            updatePlan();
        }
        robotActing_ = false;
    }

    if(!robotActing_ && msg->name != "" && msg->id != robotProgressAction_.id){
        //its a new action!
        robotActing_ = true;
        robotProgressAction_ = *msg;
        if(currentPlan_ != -1){
            checkAction(robotProgressAction_, robotName_);
            updatePlan();
        }
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
            prevId_ = currentActions[i].id;
            if(currentPlan_ != -1 ){
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
                }else if(!robotActing_ && currentActions[i].actors[0] == robotName_){
                    toTreatRobotResult_ = currentActions[i];
                }else{
                    //it is not the robot action
                    checkAction(currentActions[i], currentActions[i].actors[0]);
                    if(currentPlan_ != -1){
                        //the plan was not broked by this action
                        updatePlan();
                    }
                }
            }else if(robotActing_ && robotProgressAction_.id == currentActions[i].id){
				robotActing_ = false;
				supervisor_msgs::EndPlan srv;
                srv.request.success = false;
                srv.request.evaluate = false;
                if (!client_end_plan_->call(srv)){
                  ROS_ERROR("[plan_maintainer] Failed to call service plan_elaboration/end_plan");
                 }
                endCurrentPlan();
            }else{
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
  node_->getParam("supervisor/robot/name", robotName_);
  node_->getParam("/supervisor/AgentX", xAgent_);
  fillHighLevelNames();
  currentPlan_ = -1;
  lastPlanId_ = -1;
  prevId_ = -1;

  robotActing_ = false;
  changed_ = false;

  ros::Subscriber sub_plan = node_->subscribe("plan_elaboration/plan", 1, planCallback);
  ros::Subscriber sub_robot_action = node_->subscribe("/action_executor/current_robot_action", 1, robotActionCallback);
  ros::Subscriber sub_humans_action = node_->subscribe("mental_states/previous_actions", 1, previousActionCallback);

  ros::ServiceServer service_end = node.advertiseService("plan_maintainer/stop", stopSrv); //when a plan needs to be stopped

  ros::ServiceClient client_end_plan = node_->serviceClient<supervisor_msgs::EndPlan>("plan_elaboration/end_plan");
  client_end_plan_ = &client_end_plan;
  ros::ServiceClient client_stop_action = node_->serviceClient<std_srvs::Trigger>("robot_decision/stop");
  client_stop_action_ = &client_stop_action;


  ros::Publisher todo_actions = node_->advertise<supervisor_msgs::ActionsList>("/plan_maintainer/actions_todo", 1);
  ros::Publisher planned_actions = node_->advertise<supervisor_msgs::ActionsList>("/plan_maintainer/planned_actions", 1);
  ROS_INFO("[plan_maintainer] plan_maintainer ready");

  while (node.ok()) {
      ros::spinOnce();
      supervisor_msgs::ActionsList msg;
      msg.prevId = prevId_;
      msg.changed = changed_;
      msg.actions = todoActions_;
      todo_actions.publish(msg);
      msg.actions = plannedActions_;
      planned_actions.publish(msg);
      changed_ = false;
      loop_rate.sleep();
  }

  return 0;
}

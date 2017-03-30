/**
author Sandra Devin

Main class of the mental_states manager

**/

#include "mental_states/ms_manager.h"

ros::NodeHandle* node_;
MsManager* ms_;
bool needCheckEffect_, needCheckPrec_, needCheckGoal_;
std::vector<supervisor_msgs::Action> oldMsgPrevious_;

/**
 * \brief Callback of the database tables
 * @param msg topic msg
 * */
void dbCallback(const toaster_msgs::DatabaseTables::ConstPtr& msg){

    std::vector<toaster_msgs::DatabaseTable> tables = msg->tables;
    for(std::vector<toaster_msgs::DatabaseTable>::iterator it = tables.begin(); it != tables.end(); it++){
        if(it->changed && it->agentName != ms_->robotName_){
            needCheckEffect_ = true;
        }
    }
    ms_->agentsTable_ = tables;
}

/**
 * \brief Callback of the goal list
 * @param msg topic msg
 * */
void goalCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    if(ms_->currentRobotGoal_ != msg->currentGoal){
        needCheckGoal_ = true;
        ms_->currentRobotGoal_ = msg->currentGoal;
    }
}

/**
 * \brief Callback of the previous actions list
 * @param msg topic msg
 * */
void prevCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    if(msg->changed){
        //look for the new action added to the previous list and check if the agents are aware of the action
        std::vector<supervisor_msgs::Action> currentActions = msg->actions;
        if(currentActions.size() > oldMsgPrevious_.size()){
            //the action(s) performed are at the end of the list
            for(int i = oldMsgPrevious_.size(); i < currentActions.size(); i++){
                for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
                    //check if the agents sees the action
                    bool canSee = ms_->canSee(itms->agentName, currentActions[i].actors[0]);
                    supervisor_msgs::Action newAction;
                    if(ms_->currentRobotAction_.id == currentActions[i].id){
                        //it was the robot current action
                        newAction = ms_->currentRobotAction_;
                        newAction.succeed = currentActions[i].succeed;
                        ms_->currentRobotAction_.id = -1;
                        if(canSee){
                            itms->currentRobotAction.id = -1;
                            itms->previousActions.push_back(newAction);
                            ms_->addEffects(newAction, itms->agentName);
                        }
                    }else if(canSee){
                        if(ms_->currentPlan_.id != -1){
                            //check if the action is from the plan
                            supervisor_msgs::Action inPlanAction = ms_->isInList(currentActions[i], itms->todoActions);
                            if(inPlanAction.id != -1){
                                if(inPlanAction.actors[0] == ms_->agentX_){
                                    inPlanAction.actors[0] = currentActions[i].actors[0];
                                }
                                newAction = inPlanAction;
                                newAction.succeed = currentActions[i].succeed;
                            }
                        }else{
                            newAction = ms_->addPrecsAndEffects(currentActions[i]);
                            newAction.succeed = currentActions[i].succeed;
                        }
                        itms->previousActions.push_back(newAction);
                        ms_->addEffects(newAction, itms->agentName);
                    }
                }
            }
        }
        oldMsgPrevious_ = currentActions;
        needCheckPrec_ = true;
    }
}

/**
 * \brief Callback of the shared plan
 * @param msg topic msg
 * */
void planCallback(const supervisor_msgs::SharedPlan::ConstPtr& msg){

    if(msg->id != ms_->currentPlan_.id){
        if(ms_->currentPlan_.id != -1){
            //end current plan for all agents
            std::vector<supervisor_msgs::Action> planActions = msg->actions;
            for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
                itms->plannedActions.clear();
                itms->todoActions.clear();
                //replace by the new plans
                for(std::vector<supervisor_msgs::Action>::iterator it = planActions.begin(); it != planActions.end(); it++){
                    itms->plannedActions.push_back(ms_->addPrecsAndEffects(*it));
                }
            }
            //we check the current robot action if any
            if(ms_->currentRobotAction_.id != -1){
                supervisor_msgs::Action inPlanAction = ms_->isInList(ms_->currentRobotAction_, planActions);
                if(inPlanAction.id != -1){
                    ms_->currentRobotAction_.id = inPlanAction.id;
                    for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
                        if(itms->currentRobotAction.id != -1){
                            itms->currentRobotAction.id = inPlanAction.id;
                        }
                    }
                }
            }
        }
        needCheckPrec_ = true;
        ms_->currentPlan_ = *msg;
    }
}

/**
 * \brief Callback of the current robot action
 * @param msg topic msg
 * */
void robotActionCallback(const supervisor_msgs::Action::ConstPtr& msg){

    if(ms_->currentRobotAction_.id = msg->id){
        supervisor_msgs::Action newAction = ms_->addPrecsAndEffects(*msg);
         ms_->currentRobotAction_ = newAction;
         for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
             if(ms_->canSee(itms->agentName, ms_->robotName_)){
                 itms->currentRobotAction = newAction;
             }
         }
    }
}


/**
 * \brief Callback of the info given from dialogue node
 * @param msg topic msg
 * */
void infoCallback(const supervisor_msgs::Info::ConstPtr& msg){

    if(msg->type == "FACT"){
        if(msg->toRobot){
            ms_->addRmFactToAgent(msg->fact, ms_->robotName_, msg->isTrue);
        }
        ms_->addRmFactToAgent(msg->fact, msg->agent, msg->isTrue);
        needCheckEffect_ = true;
        needCheckPrec_ = true;
    }else if(msg->type == "ACTION"){
        for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
            if(itms->agentName == msg->agent){
                //check if the agents sees the action
                supervisor_msgs::Action newAction;
                if(ms_->currentRobotAction_.id == msg->action.id){
                    //it was the robot current action
                    newAction = ms_->currentRobotAction_;
                    newAction.succeed = msg->action.succeed;
                    ms_->currentRobotAction_.id = -1;
                    itms->currentRobotAction.id = -1;
                }
                if(ms_->currentPlan_.id != -1){
                    //check if the action is from the plan
                    supervisor_msgs::Action inPlanAction = ms_->isInList(msg->action, itms->todoActions);
                    if(inPlanAction.id != -1){
                        if(inPlanAction.actors[0] == ms_->agentX_){
                            inPlanAction.actors[0] = msg->action.actors[0];
                        }
                        newAction = inPlanAction;
                        newAction.succeed = msg->action.succeed;
                    }
                }else{
                    newAction = ms_->addPrecsAndEffects(msg->action);
                    newAction.succeed = msg->action.succeed;
                }
                itms->previousActions.push_back(newAction);
                ms_->addEffects(newAction, itms->agentName);
                break;
            }
        }
        needCheckPrec_ = true;
    }else if(msg->type == "ASK_ACTION"){
        for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
            if(itms->agentName == msg->agent){
                supervisor_msgs::Action inTodoAction = ms_->isInList(msg->action, itms->todoActions);
                if(inTodoAction.id == -1){
                    supervisor_msgs::Action toAdd = msg->action;
                    //if needed we remove if from planned actions
                    supervisor_msgs::Action inPlannedAction = ms_->isInList(msg->action, itms->plannedActions);
                    if(inPlannedAction.id != -1){
                        toAdd.id = inPlannedAction.id;
                        for(std::vector<supervisor_msgs::Action>::iterator it = itms->plannedActions.begin(); it != itms->plannedActions.end(); it++){
                            if(it->id == toAdd.id){
                                itms->plannedActions.erase(it);
                                break;
                            }
                        }
                    }
                    //we add it in the todo actions
                    itms->todoActions.push_back(toAdd);
                }
                break;
            }
        }
    }else if(msg->type == "GOAL"){
        for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
            if(itms->agentName == msg->agent){
                itms->robotGoal == msg->goal;
                std::string actorsParam = "goal_manager/goals/" + msg->goal + "_actors";
                std::vector<std::string> actors;
                node_->getParam(actorsParam, actors);
                if(ms_->isInList(actors, itms->agentName)){
                    itms->agentGoal == msg->goal;
                }
            }
        }
    }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "mental_states");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate loop_rate(30);

  MsManager ms(node_);
  ms_ = &ms;
  needCheckEffect_ = false;
  needCheckGoal_ = false;
  needCheckPrec_ = false;

  ros::Subscriber sub_db = node_->subscribe("database_manager/tables", 1, dbCallback);
  ros::Subscriber sub_goal = node_->subscribe("goal_manager/goalsList", 1, goalCallback);
  ros::Subscriber sub_prev = node_->subscribe("supervisor/previous_actions", 1, prevCallback);
  ros::Subscriber sub_plan = node_->subscribe("plan_elaboration/plan", 1, planCallback);
  ros::Subscriber robot_action = node_->subscribe("/action_executor/current_robot_action", 1, robotActionCallback);
  ros::Subscriber sub_info = node_->subscribe("/dialogue_node/infoGiven", 1, infoCallback);

  ros::Publisher ms_pub = node_->advertise<supervisor_msgs::MentalStatesList>("/mental_states/mental_states", 1);

  ROS_INFO("[mental_states] mental_states ready");

  while (node.ok()) {
      ros::spinOnce();

      bool changed  = false;
      while(needCheckEffect_ || needCheckGoal_ || needCheckPrec_){
        if(needCheckEffect_){
            needCheckPrec_ = needCheckPrec_ || ms_->checkEffects();
        }
        if(needCheckPrec_){
            needCheckEffect_ = needCheckEffect_ || ms_->checkPrecs();
        }
        if(needCheckGoal_){
            ms_->checkGoals();
        }
        changed = true;
      }

      supervisor_msgs::MentalStatesList toPublish;
      toPublish.mentalStates = ms_->msList_;
      toPublish.changed = changed;
      ms_pub.publish(toPublish);
      needCheckEffect_ = false;
      needCheckGoal_ = false;
      needCheckPrec_ = false;

      loop_rate.sleep();
  }

  return 0;
}

/**
author Sandra Devin

Main class of the mental_states manager

**/

#include "mental_states/ms_manager.h"

ros::NodeHandle* node_;
MsManager* ms_;
bool needCheckEffect_, needCheckPrec_, needCheckGoal_;
std::vector<supervisor_msgs::Action> oldMsgPrevious_;
int prevIdRobot_, prevIdHuman_;
bool infoGiven_;
std::string systemMode_, mainPartner_;
std::vector<supervisor_msgs::Action> toTell_;

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
 * \brief Callback of the previous actions list (hold system)
 * @param msg topic msg
 * */
void prevHoldCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    //look for the new action added to the previous list and check if the agents are aware of the action
    std::vector<supervisor_msgs::Action> currentActions = msg->actions;
    if(currentActions.size() > oldMsgPrevious_.size()){
        //the action(s) performed are at the end of the list
        for(int i = oldMsgPrevious_.size(); i < currentActions.size(); i++){
            if(currentActions[i].actors[0] == ms_->robotName_){
                prevIdRobot_ = currentActions[i].id;
                if(!ms_->isVisibleBy(ms_->robotName_, mainPartner_) && currentActions[i].succeed){
                    ROS_WARN("add to tell");
                    toTell_.push_back(currentActions[i]);
                }
            }else{
                prevIdHuman_ = currentActions[i].id;
            }
            if(currentActions[i].succeed){
                ms_->addEffects(ms_->addPrecsAndEffects(currentActions[i]), ms_->robotName_);
            }
        }
        oldMsgPrevious_ = currentActions;
    }
}

/**
 * \brief Callback of the previous actions list
 * @param msg topic msg
 * */
void prevCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    //look for the new action added to the previous list and check if the agents are aware of the action
    std::vector<supervisor_msgs::Action> currentActions = msg->actions;
    if(currentActions.size() > oldMsgPrevious_.size()){
        //the action(s) performed are at the end of the list
        for(int i = oldMsgPrevious_.size(); i < currentActions.size(); i++){
            if(currentActions[i].actors[0] == ms_->robotName_){
                prevIdRobot_ = currentActions[i].id;
            }else{
                prevIdHuman_ = currentActions[i].id;
            }
            supervisor_msgs::Action action = ms_->addPrecsAndEffects(currentActions[i]);
            if(action.succeed){
                ms_->addEffects(action, ms_->robotName_);
            }
            for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
                //check if the agents sees the action
                bool canSee = ms_->canSee(itms->agentName, action.actors[0]);
                supervisor_msgs::Action newAction;
                if(ms_->currentRobotAction_.id == action.id){
                    //it was the robot current action
                    newAction = action;
                    ms_->currentRobotAction_.id = -1;
                    if(canSee){
                        itms->currentRobotAction.id = -1;
                        itms->previousActions.push_back(newAction);
                        if(newAction.succeed){
                            ms_->addEffects(newAction, itms->agentName);
                        }
                    }
                }else if(canSee){
                    if(ms_->currentPlan_.id != -1){
                        //check if the action is from the plan
                        supervisor_msgs::Action inPlanAction = ms_->isInList(action, itms->todoActions);
                        if(inPlanAction.id != -1){
                            if(inPlanAction.actors[0] == ms_->agentX_){
                                inPlanAction.actors[0] = action.actors[0];
                            }
                            newAction = inPlanAction;
                            newAction.succeed = action.succeed;
                        }else{
                            newAction = action;
                        }
                    }else{
                        newAction = action;
                    }
                    if(newAction.succeed){
                        if(action.name == "pick" && newAction.name != "pick"){
                            //take pick effects
                            newAction.name = "pick";
                            newAction.effects.clear();
                            newAction.precs.clear();
                            newAction = ms_->addPrecsAndEffects(newAction);
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
        if(msg->id != -1){
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
        }else{
            //end current plan for all agents
            for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
                itms->plannedActions.clear();
                itms->todoActions.clear();
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

    //it is a new action
    if(ms_->currentRobotAction_.id != msg->id){
        supervisor_msgs::Action newAction = ms_->addPrecsAndEffects(*msg);
         ms_->currentRobotAction_ = newAction;
    }
    //we check for each agent if he is aware of the action
    for(std::vector<supervisor_msgs::MentalState>::iterator itms = ms_->msList_.begin(); itms != ms_->msList_.end(); itms++){
        if(itms->currentRobotAction.id != msg->id){
            if(ms_->canSee(itms->agentName, ms_->robotName_)){
                itms->currentRobotAction = ms_->currentRobotAction_;
            }
        }
    }
}


/**
 * \brief Callback of the info given from dialogue node
 * @param msg topic msg
 * */
void infoCallback(const supervisor_msgs::Info::ConstPtr& msg){

    infoGiven_ = true;

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
                supervisor_msgs::Action newAction;
                if(ms_->currentRobotAction_.id == msg->action.id){
                    //it was the robot current action
                    newAction = ms_->currentRobotAction_;
                    newAction.succeed = msg->action.succeed;
                    ms_->currentRobotAction_.id = -1;
                    itms->currentRobotAction.id = -1;
                }else if(ms_->currentPlan_.id != -1){
                    //check if the action is from the plan
                    supervisor_msgs::Action inPlanAction = ms_->isInList(msg->action, itms->todoActions);
                    if(inPlanAction.id == -1){
                        inPlanAction = ms_->isInList(msg->action, itms->plannedActions);
                    }
                    if(inPlanAction.id != -1){
                        if(inPlanAction.actors[0] == ms_->agentX_){
                            inPlanAction.actors[0] = msg->action.actors[0];
                        }
                        newAction = inPlanAction;
                        newAction.succeed = msg->action.succeed;
                    }else{
                        newAction = ms_->addPrecsAndEffects(msg->action);
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
                itms->robotGoal = msg->goal;
                std::string actorsParam = "goal_manager/goals/" + msg->goal + "_actors";
                std::vector<std::string> actors;
                node_->getParam(actorsParam, actors);
                if(ms_->isInList(actors, itms->agentName)){
                    itms->agentGoal = msg->goal;
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
  infoGiven_ = false;
  prevIdRobot_ =-1;
  prevIdHuman_ =-1;

  node_->getParam("/supervisor/systemMode", systemMode_);
  node_->getParam("/supervisor/mainPartner", mainPartner_);

  ros::Subscriber sub_db;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_plan;
  ros::Subscriber robot_action;
  ros::Subscriber sub_info;
  ros::Subscriber sub_prev;
  if(systemMode_ == "new"){
      sub_db = node_->subscribe("database_manager/tables", 1, dbCallback);
      sub_goal = node_->subscribe("goal_manager/goalsList", 1, goalCallback);
      sub_plan = node_->subscribe("plan_elaboration/plan", 1, planCallback);
      robot_action = node_->subscribe("/action_executor/current_robot_action", 1, robotActionCallback);
      sub_info = node_->subscribe("/dialogue_node/infoGiven", 1, infoCallback);
      sub_prev = node_->subscribe("supervisor/previous_actions", 1, prevCallback);
  }else{
      sub_prev = node_->subscribe("supervisor/previous_actions", 1, prevHoldCallback);
  }

  ros::Publisher ms_pub = node_->advertise<supervisor_msgs::MentalStatesList>("/mental_states/mental_states", 1);

  ROS_INFO("[mental_states] mental_states ready");

  while (node.ok()) {
      ros::spinOnce();

      bool changed  = false;
      while(needCheckEffect_ || needCheckGoal_ || needCheckPrec_){
        if(needCheckEffect_){
            needCheckPrec_ = needCheckPrec_ || ms_->checkEffects();
            needCheckEffect_ = false;
        }
        if(needCheckPrec_){
            needCheckEffect_ = needCheckEffect_ || ms_->checkPrecs();
            needCheckPrec_ = false;
        }
        if(needCheckGoal_){
            ms_->checkGoals();
            needCheckGoal_ = false;
        }
        changed = true;
      }
      
      if(toTell_.size()>0){
	ROS_WARN("to tell published");
      }

      supervisor_msgs::MentalStatesList toPublish;
      toPublish.mentalStates = ms_->msList_;
      toPublish.changed = changed;
      toPublish.prevIdRobot = prevIdRobot_;
      toPublish.prevIdHuman = prevIdHuman_;
      toPublish.infoGiven = infoGiven_;
      toPublish.toTell = toTell_;
      infoGiven_ = false;
      ms_pub.publish(toPublish);
      if(toTell_.size()>0){
        ROS_WARN("to publish size: %d", toPublish.toTell.size());
      }
      toTell_.clear();
      loop_rate.sleep();
  }

  return 0;
}

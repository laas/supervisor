/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include "action_executor/action_executor.h"

ros::NodeHandle* node_;
ActionExecutor* executor_;
std::string humanSafetyJoint_;
double safetyThreshold_, toWatchThreshold_;
bool started = false;
std::string condition_;
int seed;

/**
 * \brief Service to stop the current action
 * @param req empty request
 * @param res empty result
 * @return true
 * */
bool stopOrder(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){

   if(executor_->connector_.isActing_){
    executor_->connector_.stopOrder_ = true;
   }

   return true;
}

/**
 * \brief Callback for agent monitor facts
 * @param msg the list of facts
 * */
void agentFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){

    std::vector<toaster_msgs::Fact> distanceFacts = msg->factList;
    for(std::vector<toaster_msgs::Fact>::iterator it = distanceFacts.begin(); it != distanceFacts.end(); it++){
        if(it->property == "Distance"){
            if(it->subjectId == humanSafetyJoint_){
                //keep distance from object to the human safety joints
                executor_->connector_.humanDistances_[it->targetId] = it->doubleValue;
                if(executor_->connector_.isActing_){
                    if(it->targetId == executor_->connector_.robotToaster_){
                        //check if the robot should stop for safety reasons
                        if(it->doubleValue < safetyThreshold_){
                            executor_->connector_.stopOrder_ = true;
                        }else{
                            executor_->connector_.stopOrder_ = false;
                        }
                    /*}else if(it->targetId == executor_->connector_.objectToWatch_){
                        //check if the robot should change its object refinement
                        if(it->doubleValue < toWatchThreshold_){
                            executor_->connector_.refineOrder_ = true;
                        }else{
                            executor_->connector_.refineOrder_ = false;
                        }*/
                    }
                }
            }
        }
    }
}

/**
 * \brief Callback of human actions
 * @param msg the human action executed
 * */
void humanActionCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    //when an action is executed by a human, we reset the gtp previous id
    executor_->connector_.previousId_ = -1;
    supervisor_msgs::Action humanAction = msg->actions[0];
    if(humanAction.name == "place"){
	for(int i = 0; i < humanAction.parameter_keys.size(); i++){
		if(humanAction.parameter_keys[i] == "support"){
			if(humanAction.parameter_values[i] == executor_->connector_.objectToWatch_){
				executor_->connector_.refineOrder_ = true;
				ROS_WARN("refine order");
			}
			break;
		}
	}
    }else if(humanAction.name == "pick"){
        for(int i = 0; i < humanAction.parameter_keys.size(); i++){
                if(humanAction.parameter_keys[i] == "object"){
                       if(humanAction.parameter_values[i] == executor_->connector_.onScanArea1_){
                            executor_->connector_.onScanArea1_ = "NONE";
                       }
                       if(humanAction.parameter_values[i] == executor_->connector_.onScanArea2_){
                            executor_->connector_.onScanArea2_ = "NONE";
                       }
			if(humanAction.parameter_values[i] == "RED_TAPE2" && executor_->connector_.objectToWatch_ == "RED_TAPE"){
				executor_->connector_.stopOrder_ = true;
				break;
			}
                        if(humanAction.parameter_values[i] == executor_->connector_.objectToWatch_){
                                executor_->connector_.stopOrder_ = true;
                        }
                        break;
                }
        }
    }else if(humanAction.name == "place"){
        std::string object, support;
        for(int i = 0; i < humanAction.parameter_keys.size(); i++){
            if(humanAction.parameter_keys[i] == "object"){
                object = humanAction.parameter_values[i];
            }
            if(humanAction.parameter_keys[i] == "support"){
                support = humanAction.parameter_values[i];
            }
        }
        if(support == "SCAN_AREA1"){
            executor_->connector_.onScanArea1_ = object;
        }
        if(support == "SCAN_AREA2"){
            executor_->connector_.onScanArea2_ = object;
        }
    }
}

/**
 * \brief Callback of human actions
 * @param msg the human action executed
 * */
void gtpTrajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg){

    executor_->connector_.curTraj_ = *msg;
    executor_->connector_.needTraj_ = false;
}

/**
 * \brief Callback of the goal list
 * @param msg topic msg
 * */
void goalsListCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    if(msg->changed){
        if(msg->currentGoal == "SCAN_US" && !started){
            started = true;
            executor_->connector_.nbConflicts_ = 0;
            seed++;
            srand(seed);
        }
        if(msg->currentGoal == "NONE" && started){
            started = false;
            //we log results
            std::ofstream fileSave;
            //we save the execution time
            std::string fileName = "/home/sdevin/catkin_ws/src/supervisor/logs/Conflicts.txt";
            fileSave.open(fileName.c_str(), std::ios::app);
            std::ostringstream strs;
            strs << executor_->connector_.nbConflicts_;
            std::string nbConf = strs.str();
            fileSave << "\t Condition " << condition_.c_str() << "\t " << nbConf.c_str() << std::endl;
        }
    }
}

/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate loop_rate(30);

  ROS_INFO("[action_executor] Init");

  node.getParam("/action_executor/humanSafetyJoint", humanSafetyJoint_);
  node.getParam("/action_executor/safetyThreshold", safetyThreshold_);
  node.getParam("/action_executor/toWatchThreshold", toWatchThreshold_);

  node_->getParam("scan_simu1/seed", seed);
  srand (seed);

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

  ActionExecutor executor("supervisor/action_executor", node_);
  executor_ = &executor;

  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1, agentFactListCallback);
  ros::Subscriber sub_human_action = node.subscribe("human_monitor/current_humans_action", 100, humanActionCallback);
  ros::Subscriber sub_gtp_traj = node.subscribe("/gtp/ros_trajectory", 10, gtpTrajCallback);
  ros::Subscriber sub_goal = node.subscribe("/goal_manager/goalsList", 10, goalsListCallback);

  ros::ServiceServer service_stop = node.advertiseService("action_executor/stop", stopOrder); //stop the execution

  ROS_INFO("[action_executor] Ready");

  while(node.ok()){
      ros::spinOnce();
      //publish the current action
      if(executor.connector_.isActing_ && executor.connector_.currentAction_.name != "moveTo" && executor.connector_.currentAction_.name != ""){
          executor_->current_pub_.publish(executor.connector_.currentAction_);
      }
      loop_rate.sleep();
  }
}

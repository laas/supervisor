/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * **********/

#include "action_executor/action_executor.h"

ros::NodeHandle* node_;
ActionExecutor* executor_;
std::string humanSafetyJoint_;
double safetyThreshold_, toWatchThreshold_;

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

    if(executor_->connector_.isActing_){
        std::vector<toaster_msgs::Fact> distanceFacts = msg->factList;
        for(std::vector<toaster_msgs::Fact>::iterator it = distanceFacts.begin(); it != distanceFacts.end(); it++){
            if(it->property == "Distance"){
                if(it->subjectId == humanSafetyJoint_){
                    executor_->connector_.humanDistances_[it->targetId] = it->doubleValue;
                    if(it->targetId == executor_->connector_.robotToaster_){
                        if(it->doubleValue < safetyThreshold_){
                            executor_->connector_.stopOrder_ = true;
                        }else{
                            executor_->connector_.stopOrder_ = false;
                        }
                    }else if(it->targetId == executor_->connector_.objectToWatch_){
                        if(it->doubleValue < toWatchThreshold_){
                            executor_->connector_.refineOrder_ = true;
                        }else{
                            executor_->connector_.refineOrder_ = false;
                        }
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

  ActionExecutor executor("supervisor/action_executor", node_);
  executor_ = &executor;

  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1, agentFactListCallback);
  ros::Subscriber sub_human_action = node.subscribe("human_monitor/current_humans_action", 1, humanActionCallback);

  ros::Publisher current_pub_ = node_->advertise<supervisor_msgs::Action>("/action_executor/current_robot_action", 1);

  ros::ServiceServer service_stop = node.advertiseService("action_executor/stop", stopOrder); //stop the execution

  ROS_INFO("[action_executor] Ready");

  while(node.ok()){
      ros::spinOnce();
      if(executor.connector_.isActing_){
          current_pub_.publish(executor.connector_.currentAction_);
      }
      loop_rate.sleep();
  }
}

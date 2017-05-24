/**
author Sandra Devin

Main class of the plan elaboration.

The plan elaboration allows to find a plan to execute a goal

**/

#include "plan_elaboration/plan_elaboration.h"


ros::NodeHandle* node_;
PlanElaboration* pe_;
supervisor_msgs::SharedPlan currentPlan_;
bool hasPlan_;
ros::ServiceClient* client_goal_;
ros::ServiceClient* client_stop_plan_;
bool needPlan_;

/**
 * \brief Callback of the goal list
 * @param msg topic msg
 * */
void goalsListCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    if(msg->changed){
        if(pe_->currentGoal_ == "NONE" && msg->currentGoal != "NONE" && msg->currentGoal != "STOP"){
            //a new goal arrived
            pe_->currentGoal_ = msg->currentGoal;
            pe_->domainInitialized_ = false;
            //look for a plan
            std::pair<bool, supervisor_msgs::SharedPlan> plan = pe_->findPlan();
            if(plan.first){
                //publish the plan
                currentPlan_ = plan.second;
                hasPlan_ = true;
            }else{
                //return failure to the goal manager
                supervisor_msgs::String srv;
                srv.request.data = pe_->currentGoal_;
                if (!client_goal_->call(srv)){
                   ROS_ERROR("[plan_elaboration] Failed to call service goal_manager/end_goal");
                }
                pe_->currentGoal_ = "NONE";
            }
        }else if(pe_->currentGoal_ != "NONE" && msg->currentGoal == "STOP"){
            //stop order: transmit it to the plan maintainer
            std_srvs::Trigger srv_stop;
            supervisor_msgs::String srv;
            if (client_stop_plan_->call(srv_stop)){
                if(srv_stop.response.success){
                    srv.request.data = "OK";
                }else{
                    srv.request.data = pe_->currentGoal_;
                }
            }else{
                ROS_ERROR("[plan_elaboration] Failed to call service plan_maintainer/stop");
                srv.request.data = pe_->currentGoal_;
            }
            if (!client_goal_->call(srv)){
               ROS_ERROR("[plan_elaboration] Failed to call service goal_manager/end_goal");
            }
            pe_->currentGoal_ = "NONE";
        }
    }
}

/**
 * \brief Service call when a plan ends
 * @param req the request of the service
 * @param res nothing to fill
 * @return true
 * */
bool endPlan(supervisor_msgs::EndPlan::Request  &req, supervisor_msgs::EndPlan::Response &res){

    if(req.success){
        //tell to the goal manager that the plan succeed
        supervisor_msgs::String srv;
        srv.request.data = "OK";
        if (!client_goal_->call(srv)){
           ROS_ERROR("[plan_elaboration] Failed to call service goal_manager/end_goal");
        }
        pe_->currentGoal_ = "NONE";
        return true;
    }

    if(req.evaluate){
        //the plan needs to be evaluate
        ROS_WARN("new plan with %s locked for %s", req.objectLocked.c_str(), req.agentLocked.c_str());
        pe_->dom_->objectLocked_ = pe_->getLockedObject(req.objectLocked, req.agentLocked);
        pe_->dom_->agentLocked_ = req.agentLocked;
    }

    if(req.forgiveAction){
        //the plan needs to be evaluate
        pe_->dom_->objectForgive_ = req.objectLocked;
        pe_->dom_->agentForgive_ = req.agentLocked;
    }

    //we look for a new plan
    needPlan_ = true;

    return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "plan_elaboration");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate loop_rate(30);

  ROS_INFO("[plan_elaboration] Init");

  PlanElaboration pe(node_);
  pe_ = &pe;


  hasPlan_ = false;
  needPlan_ = false;

  ros::Subscriber sub_goal = node.subscribe("/goal_manager/goalsList", 1, goalsListCallback);

  ros::ServiceServer service_end = node.advertiseService("plan_elaboration/end_plan", endPlan); //when a plan ends

  ros::Publisher plan_pub = node.advertise<supervisor_msgs::SharedPlan>("plan_elaboration/plan", 1);

  ros::ServiceClient client_goal = node_->serviceClient<supervisor_msgs::String>("goal_manager/end_goal");
  client_goal_ = &client_goal;
  ros::ServiceClient client_stop_plan = node_->serviceClient<std_srvs::Trigger>("plan_maintainer/stop");
  client_stop_plan_ = &client_stop_plan;


  ROS_INFO("[plan_elaboration] Ready");

  while(node.ok()){
    ros::spinOnce();
    if(needPlan_){
        std::pair<bool, supervisor_msgs::SharedPlan> plan = pe_->findPlan();
        if(plan.first){
            //publish the plan
            currentPlan_ = plan.second;
        }else{
            hasPlan_ = false;
            //return failure to the goal manager
            supervisor_msgs::String srv;
            srv.request.data = pe_->currentGoal_;
            if (!client_goal_->call(srv)){
               ROS_ERROR("[plan_elaboration] Failed to call service goal_manager/end_goal");
            }
            pe_->currentGoal_ = "NONE";
        }
        needPlan_ = false;
    }
    if(hasPlan_){
        plan_pub.publish(currentPlan_);
    }
    loop_rate.sleep();
  }
 
  ros::spin();

  return 0;
}

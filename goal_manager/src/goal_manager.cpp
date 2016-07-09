/**
author Sandra Devin

**/

#include <goal_manager/goal_manager.h>


GoalManager::GoalManager(ros::NodeHandle* node){
   currentGoal_ = "NONE";
   node_ = node;
}

/*
New goal to manage
*/
void GoalManager::addGoal(string goal){
   
      waitingGoals_.push(goal);
}
/*
When a goal is over, look if there is another goal to execute
*/
void GoalManager::chooseGoal(){

   ros::ServiceClient client = node_->serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
   ros::ServiceClient clientNG = node_->serviceClient<supervisor_msgs::NewGoal>("plan_elaboration/new_goal");

   if(currentGoal_ == "NONE" && waitingGoals_.size() >0){//there at least one waiting goal and no current goal
      string newGoal = waitingGoals_.front();
      waitingGoals_.pop();
      supervisor_msgs::ChangeState srv;
      srv.request.type = "goal";
      srv.request.state = "PROGRESS";
	   srv.request.goal = newGoal;
	   if (!client.call(srv)){
       ROS_ERROR("[goal_manager] Failed to call service mental_state/change_state");
	   }
	   //And we execute it
      currentGoal_ = newGoal;
      ROS_INFO("[goal_manager] Executing the goal %s", newGoal.c_str());
      //TODO: send goal to plan_elaboration
      supervisor_msgs::NewGoal srvNG;
      srvNG.request.goal = currentGoal_;
      if (!clientNG.call(srvNG)){
      ROS_ERROR("[goal_manager] Failed to call service plan_elaboration/new_goal");
      }
   }
}

/*
Function call when the plan ends
 @report: true if the goal has been achieved
*/
void GoalManager::endGoal(bool report){

   ros::ServiceClient clientCS = node_->serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
   supervisor_msgs::ChangeState serviceCS;
   if(!report){//if the goal has been achieved, no need to send info to the MS manager (it should know this by itself)
       ROS_INFO("[goal_manager] Aborting the goal %s", currentGoal_.c_str());
       string robotName;
       node_->getParam("/robot/name", robotName);
       serviceCS.request.type = "goal";
       serviceCS.request.state = "ABORT";
       serviceCS.request.agent = robotName;
       serviceCS.request.goal = currentGoal_;
       if(!clientCS.call(serviceCS)){
           ROS_ERROR("Failed to call service mental_state/change_state");
       }
   }
   currentGoal_ = "NONE";
}

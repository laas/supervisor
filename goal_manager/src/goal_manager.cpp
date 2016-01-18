/**
author Sandra Devin

**/

#include <goal_manager/goal_manager.h>


GoalManager::GoalManager(){
   currentGoal_ = "NONE";
}

/*
New goal to manage
*/
void GoalManager::addGoal(string goal){

   ros::NodeHandle node;
   ros::ServiceClient client = node.serviceClient<supervisor_msgs::NewGoal>("mental_state/start_goal");

   if(currentGoal_ == "NONE"){//if there is no current goal, we execute the goal
      //We say to the mental state manager that we start the goal
      supervisor_msgs::NewGoal srv;
	   srv.request.goal = goal;
	   if (!client.call(srv)){
	   ROS_ERROR("[goal_manager] Failed to call service mental_state/start_goal");
	   }
	   //And we execute it
      currentGoal_ = goal;
      ROS_INFO("[goal_manager] Executing the goal %s", goal.c_str());
      executeGoal(goal);
   }else{//else we put it in the waiting queue
      waitingGoals_.push(goal);
   }
}

/*
Executes a goal: compute facts, look for a plan and execute it if found, else failed the goal.
*/
void GoalManager::executeGoal(string goal){
	
	//TODO:
	//If needed, we compute the needed facts
	
	//Then we ask to hatp a plan
	
	//if there is a plan we execute it (give the new plan to the ms manager)
	//else, we fail the goal
}

/**
author Sandra Devin

Main class of the goal_manager.

The goal manager allows to choose a goal to execute and find a plan to execute it

**/

#include <goal_manager/goal_manager.h>

GoalManager* gm = new GoalManager();

/*
Service call to execute a new goal
*/
bool newGoal(supervisor_msgs::NewGoal::Request  &req, supervisor_msgs::NewGoal::Response &res){
	
	ros::NodeHandle node;
   ros::ServiceClient client = node.serviceClient<supervisor_msgs::NewGoal>("mental_state/new_goal");
   
	//We say to the mental state manager that we have a new goal
   supervisor_msgs::NewGoal srv;
	srv.request.goal = req.goal;
	if (!client.call(srv)){
	 ROS_ERROR("[goal_manager] Failed to call service mental_state/new_goal");
	}
		
	//Then we manage the goal
	gm->addGoal(req.goal);

	return true;
}
/*
Service call when the plan is over
*/
bool endPlan(supervisor_msgs::EndPlan::Request  &req, supervisor_msgs::EndPlan::Response &res){
   
   gm->endPlan(req.report);

	return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "goal_manager");
  ros::NodeHandle node;
  	ros::Rate loop_rate(30);

  ROS_INFO("[goal_manager] Init goal_manager");
 
  //Services declarations
  ros::ServiceServer service_goal = node.advertiseService("goal_manager/new_goal", newGoal); //new goal to execute
  ros::ServiceServer end_plan = node.advertiseService("goal_manager/end_plan", endPlan); //the plan is over

  ROS_INFO("[goal_manager] goal_manager ready");
  while(node.ok()){
   gm->chooseGoal();
   ros::spinOnce();
  	loop_rate.sleep();
  }

  return 0;
}

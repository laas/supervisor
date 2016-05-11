/**
author Sandra Devin

Main class of the goal_manager.

The goal manager allows to choose a goal to execute

**/

#include <goal_manager/goal_manager.h>

GoalManager* gm;
ros::NodeHandle* node;

/*
Service call to execute a new goal
*/
bool newGoal(supervisor_msgs::NewGoal::Request  &req, supervisor_msgs::NewGoal::Response &res){

   ros::ServiceClient client = node->serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
   
	//We say to the mental state manager that we have a new goal
   supervisor_msgs::ChangeState srv;
   srv.request.type = "goal";
   srv.request.state = "NEW";
	srv.request.goal = req.goal;
	if (!client.call(srv)){
     ROS_ERROR("[goal_manager] Failed to call service mental_state/change_state");
	}
		
	//Then we manage the goal
	gm->addGoal(req.goal);

	return true;
}

/*
Service call when the plan is over
*/
bool endGoal(supervisor_msgs::EndPlan::Request  &req, supervisor_msgs::EndPlan::Response &res){
   
   gm->endGoal(req.report);

	return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "goal_manager");
  ros::NodeHandle _node;
  node = &_node;
  ros::Rate loop_rate(30);

  GoalManager _gm(&_node);
  gm = &_gm;

  ROS_INFO("[goal_manager] Init goal_manager");
 
  //Services declarations
  ros::ServiceServer service_goal = _node.advertiseService("goal_manager/new_goal", newGoal); //new goal to execute
  ros::ServiceServer end_goal = _node.advertiseService("goal_manager/end_goal", endGoal); //the plan is over

  ROS_INFO("[goal_manager] goal_manager ready");
  while(_node.ok()){
   gm->chooseGoal();
   ros::spinOnce();
  	loop_rate.sleep();
  }

  return 0;
}

/**
author Sandra Devin

Main class of the mental_state manager.

The mental state manager estimates and maintains the mental states of each agent concerning goals, plans and actions.

**/

#include <mental_states/ms_manager.h>

/*
Function which updates the mental state of an agent
	@agent: the name of the agent
*/

MSManager* ms = new MSManager();
DBInterface* db = new DBInterface();
string robot_name;
vector<string> other_agents;

/*
Service call when we ask the robot to execute a new goal
*/
bool newGoal(supervisor_msgs::NewGoal::Request  &req, supervisor_msgs::NewGoal::Response &res){
		
	supervisor_msgs::GoalMS* goal = NULL;
	//Get the goal by its name
	goal = ms->getGoalByName(req.goal);
	if(!goal){
		ROS_ERROR("[mental_state] Unknown goal");
		return false;
	}

	//add the goal state to PENDING into the knowledge of each of its actor
	vector<string> actors = goal->actors;
	for(vector<string>::iterator it = actors.begin(); it != actors.end(); it++){
		db->addGoalState(*goal, *it, "PENDING");
	}
	return true;
	
}


/*
Service call when we the robot starts to execute a goal
*/
bool startGoal(supervisor_msgs::StartGoal::Request  &req, supervisor_msgs::StartGoal::Response &res){
	
	supervisor_msgs::GoalMS* goal = NULL;
	//Get the goal by its name
	goal = ms->getGoalByName(req.goal);
	if(!goal){
		ROS_ERROR("[mental_state] Unknown goal");
		return false;
	}

	//add the goal state to PROGRESS into the knowledge of each of its actor
	vector<string> actors = goal->actors;
	for(vector<string>::iterator it = actors.begin(); it != actors.end(); it++){
		db->addGoalState(*goal, *it, "PROGRESS");
	}
	return true;
}

/*
Service call when a plan is computed by the robot
*/
bool newPlan(supervisor_msgs::NewPlan::Request  &req, supervisor_msgs::NewPlan::Response &res){
	
	//Transform the plan into a PlanMS
	supervisor_msgs::PlanMS plan;
	plan.goal = *(ms->getGoalByName(req.plan.goal));
	plan.id = ms->getAndIncreasePlanId();
	plan.links = req.plan.links;
	for(vector<supervisor_msgs::Action>::iterator it = req.plan.actions.begin(); it != req.plan.actions.end(); it++){
		supervisor_msgs::ActionMS action = ms->createActionFromHighLevel(*it);
		plan.actions.push_back(action);
	}

	//add the plan state to PROGRESS and the action to PLANNED into the robot knowledge and both to UNKNOWN into the other agent knowledge
	for(vector<string>::iterator it = other_agents.begin(); it != other_agents.end(); it++){
		if(*it == robot_name){
			db->addPlanState(plan, *it, "PROGRESS");
			for(vector<supervisor_msgs::ActionMS>::iterator ita = plan.actions.begin(); ita != plan.actions.end(); ita++){
				db->addActionState(*ita, *it, "PLANNED");
			}
		}else{
			db->addPlanState(plan, *it, "UNKNOWN");
			for(vector<supervisor_msgs::ActionMS>::iterator ita = plan.actions.begin(); ita != plan.actions.end(); ita++){
				db->addActionState(*ita, *it, "UNKNOWN");
			}
		}
	}


	return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "mental_state");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  ROS_INFO("[mental_state] Init mental_state");

  //Services declarations
  ros::ServiceServer service_new_goal = node.advertiseService("mental_state/new_goal", newGoal); //to add a new goal (state PENDING)
  ros::ServiceServer service_start_goal = node.advertiseService("mental_state/start_goal", startGoal); //to start a goal
  ros::ServiceServer service_new_plan = node.advertiseService("mental_state/new_plan", newPlan); //to add a new plan (in PROGRESS for the robot and UNKNOWN for others)

  node.getParam("/robot/name", robot_name);
  other_agents = db->getAgents();
  ms->initGoals();
  ms->initHighLevelActions();

  ROS_INFO("[mental_state] mental_state ready");

  while (node.ok()) {

  for(vector<string>::iterator it = other_agents.begin(); it != other_agents.end(); it++){
	ms->update(*it);
  }

  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}

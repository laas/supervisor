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
vector<string> all_agents;

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
	map<int, int> lmap;
	for(vector<supervisor_msgs::Action>::iterator it = req.plan.actions.begin(); it != req.plan.actions.end(); it++){
		supervisor_msgs::ActionMS action = ms->createActionFromHighLevel(*it);
		plan.actions.push_back(action);
		lmap[it->id] = action.id;
	}
	for(vector<supervisor_msgs::Link>::iterator it = req.plan.links.begin(); it != req.plan.links.end(); it++){
		supervisor_msgs::Link new_link;
		new_link.origin = lmap[it->origin];
		new_link.following = lmap[it->following];
		plan.links.push_back(new_link);
	}
	
	//Add the plan to the list of plans
	ms->addPlanToList(plan);

	//add the plan state to PROGRESS and the action to PLANNED into the robot knowledge and both to UNKNOWN into the other agent knowledge
	for(vector<string>::iterator it = all_agents.begin(); it != all_agents.end(); it++){
		if(*it == robot_name){
			db->addActionsState(plan.actions, *it, "PLANNED");
			db->addPlanState(plan, *it, "PROGRESS");
		}else{
			db->addActionsState(plan.actions, *it, "UNKNOWN");
			db->addPlanState(plan, *it, "UNKNOWN");
		}
	}


	return true;
}

/*
Service call to abort the current plan for an agent
*/
bool abortPlan(supervisor_msgs::AbortPlan::Request  &req, supervisor_msgs::AbortPlan::Response &res){
	
	ms->abortPlan(req.agent);


	return true;
}

/*
Service call to abort the robot shares the current plan
*/
bool sharePlan(supervisor_msgs::SharePlan::Request  &req, supervisor_msgs::SharePlan::Response &res){
	
	//we get all the agents which can see the robot
	vector<string> present_agents = db->getAgentsWhoSee(robot_name);
	//We get the current plan
	pair<bool, supervisor_msgs::PlanMS> robotPlan = ms->getAgentPlan(robot_name);
	if(!robotPlan.first){
		return true;
	}

	//For all these agents, the plan is now in PROGRESS and its actions PLANNED
	for(vector<string>::iterator it = present_agents.begin(); it != present_agents.end(); it++){
		db->addActionsState(robotPlan.second.actions, *it, "PLANNED");
		db->addPlanState(robotPlan.second, *it, "PROGRESS");
	}

	return true;
}

/*
Service call when the robot detects the change of state of an action (either action in PROGRESS, DONE or ABORTED)
*/
bool actionState(supervisor_msgs::ActionState::Request  &req, supervisor_msgs::ActionState::Response &res){
	
	//first we get the corresponding action
	pair<bool, supervisor_msgs::ActionMS> action = ms->getActionFromAction(req.action);

	//then we get all the agent which can see the actors of the action
	vector<string> canSee_agents = all_agents;
	for(vector<string>::iterator it = action.second.actors.begin(); it != action.second.actors.end(); it++){
		vector<string> temp_agents = db->getAgentsWhoSee(*it);
		vector<string> temp(100);
		sort(temp_agents.begin(), temp_agents.end());
		sort(canSee_agents.begin(), canSee_agents.end());
		vector<string>::iterator i = set_intersection(temp_agents.begin(), temp_agents.end(), canSee_agents.begin(), canSee_agents.end(), temp.begin());
		temp.resize(i-temp.begin());       
		canSee_agents = temp;
	}
	canSee_agents.insert(canSee_agents.end(), action.second.actors.begin(), action.second.actors.end());
	
	//for all these agent and for the actors we change the state of the action
	for(vector<string>::iterator it = canSee_agents.begin(); it != canSee_agents.end(); it++){
		vector<supervisor_msgs::ActionMS> actions;
		actions.push_back(action.second);
		db->addActionsState(actions, *it, req.state);
		if(req.state == "DONE"){//if the state is done, we also add the effects of the action in the agent knowledge
			db->addFacts(action.second.effects, *it);
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
  ros::ServiceServer service_abort_plan = node.advertiseService("mental_state/abort_plan", abortPlan); //abort the current plan for an agent
  ros::ServiceServer service_share_plan = node.advertiseService("mental_state/share_plan", sharePlan); //the robot shares the current plan
  ros::ServiceServer service_action_state = node.advertiseService("mental_state/action_state", actionState); //to change the state of an action

  node.getParam("/robot/name", robot_name);
  all_agents = db->getAgents();
  ms->initGoals();
  ms->initHighLevelActions();

  ROS_INFO("[mental_state] mental_state ready");

  while (node.ok()) {

  for(vector<string>::iterator it = all_agents.begin(); it != all_agents.end(); it++){
	ms->update(*it);
  }

  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}

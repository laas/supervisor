/**
author Sandra Devin

Main class of the mental_state manager.

The mental state manager estimates and maintains the mental states of each agent concerning goals, plans and actions.

TODO: - weak achievement goal and plan
      - similar actions: ignore DONE and FAILED actions in actionState: changeActionStateFunction?

**/

#include <mental_states/ms_manager.h>

MSManager* ms = new MSManager();
DBInterface* db = new DBInterface();
string robotName;
vector<string> allAgents;
bool simu;

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
		supervisor_msgs::Link newLink;
		newLink.origin = lmap[it->origin];
		newLink.following = lmap[it->following];
		plan.links.push_back(newLink);
	}
	
	//Add the plan to the list of plans
	ms->addPlanToList(plan);

	//add the plan state to PROGRESS and the action to PLANNED into the robot knowledge and both to UNKNOWN into the other agent knowledge
	for(vector<string>::iterator it = allAgents.begin(); it != allAgents.end(); it++){
		if(*it == robotName){
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
	vector<string> presentAgents = db->getAgentsWhoSee(robotName);
	//We get the current plan
	pair<bool, supervisor_msgs::PlanMS> robotPlan = ms->getAgentPlan(robotName);
	if(!robotPlan.first){
		return true;
	}

	//For all these agents, the plan is now in PROGRESS and its actions PLANNED
	for(vector<string>::iterator it = presentAgents.begin(); it != presentAgents.end(); it++){
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
	pair<bool, supervisor_msgs::ActionMS> actionFind = ms->getActionFromAction(req.action);
	supervisor_msgs::ActionMS action;
	if(actionFind.first){
		action = actionFind.second;
	}else {
		action = ms->createActionFromHighLevel(req.action);
	}

	//then we get all the agent which can see the actors of the action
	vector<string> canSeeAgents = allAgents;
	for(vector<string>::iterator it = action.actors.begin(); it != action.actors.end(); it++){
		vector<string> tempAgents = db->getAgentsWhoSee(*it);
		vector<string> temp(100);
		sort(tempAgents.begin(), tempAgents.end());
		sort(canSeeAgents.begin(), canSeeAgents.end());
		vector<string>::iterator i = set_intersection(tempAgents.begin(), tempAgents.end(), canSeeAgents.begin(), canSeeAgents.end(), temp.begin());
		temp.resize(i-temp.begin());       
		canSeeAgents = temp;
	}
	canSeeAgents.insert(canSeeAgents.end(), action.actors.begin(), action.actors.end());
	canSeeAgents.push_back(robotName);
	
	//for all these agent and for the actors we change the state of the action
	for(vector<string>::iterator it = canSeeAgents.begin(); it != canSeeAgents.end(); it++){
		if(req.state == "DONE"){//if the state is done, we also add the effects of the action in the agent knowledge
			db->addEffects(action.effects, *it);
		}		
		vector<supervisor_msgs::ActionMS> actions;
		actions.push_back(action);
		db->addActionsState(actions, *it, req.state);
	}

	return true;
	 
}

/*
Service call to abort a goal for an agent
*/
bool abortGoal(supervisor_msgs::AbortGoal::Request  &req, supervisor_msgs::AbortGoal::Response &res){
	
	supervisor_msgs::GoalMS* goal = NULL;
	//Get the goal by its name
	goal = ms->getGoalByName(req.goal);
	if(!goal){
		ROS_ERROR("[mental_state] Unknown goal");
		return false;
	}

	db->addGoalState(*goal, req.agent, "ABORTED");


	return true;
}

/*
Service when an agent informs an other agent about something, it can be:
	- the state of a goal, @infoType = goalState
	- the state of a plan, @infoType = planState
	- the state of a action, @infoType = actionState
	- a fact, @infoType = fact
*/
bool infoGiven(supervisor_msgs::InfoGiven::Request  &req, supervisor_msgs::InfoGiven::Response &res){
	
	if(req.infoType == "goalState"){//we put the state of the goal into the receiver and sender knowledge
		supervisor_msgs::GoalMS* goal = NULL;
		goal = ms->getGoalByName(req.goal);
		if(!goal){
			ROS_ERROR("[mental_state] Unknown goal");
			return false;
		}
		db->addGoalState(*goal, req.receiver, req.state);
		db->addGoalState(*goal, req.sender, req.state);
	}else if(req.infoType == "planState"){//we put the state of the plan into the receiver and sender knowledge
		pair<bool, supervisor_msgs::PlanMS> plan = ms->getPlanFromId(req.planId);
		if(plan.first){
			db->addPlanState(plan.second, req.receiver, req.state);
			db->addPlanState(plan.second, req.sender, req.state);
		}
	}else if(req.infoType == "actionState"){//we put the state of the action into the receiver and sender knowledge
		pair<bool, supervisor_msgs::ActionMS> action = ms->getActionFromId(req.actionId);
		if(action.first){
			vector<supervisor_msgs::ActionMS> actions;
			actions.push_back(action.second);
			db->addActionsState(actions, req.receiver, req.state);
			db->addActionsState(actions, req.sender, req.state);
		}

	}else if(req.infoType == "fact"){//we put the fact into the receiver and sender knowledge
		vector<toaster_msgs::Fact> facts;
		facts.push_back(req.fact);
		db->addFacts(facts, req.receiver);
		db->addFacts(facts, req.sender);
	}else {
		ROS_ERROR("[mental_state] Unknown infoType");
		return false;
	}


	return true;
}

/*
Servive call to get the action an agent thinks an actor has to do
*/
bool getActionTodo(supervisor_msgs::GetActionTodo::Request  &req, supervisor_msgs::GetActionTodo::Response &res){
	
	bool actionFind = false;

	//First we look for the action ASKED to the actor in the knowledge of the agent
	vector<int> askedIds = db->getActionsIdFromState(req.agent, "ASKED");
	vector<supervisor_msgs::ActionMS> askedActions = ms->getActionsFromIds(askedIds);
	for(vector<supervisor_msgs::ActionMS>::iterator it = askedActions.begin(); it != askedActions.end(); it++){
		bool isActor = false;
		for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
			if(*ita == req.actor){
				isActor = true;
				break;
			}
		}
		if(isActor){//if we find an action ASKED to the actor, we look if the agent think the action is possible (preconditions)
			if(db->factsAreIn(req.agent, it->prec)){
				//if the action is possible we return this action
				res.state = "READY"; //we return READY in order to inform that the action is possible
				res.action = ms->convertActionMStoAction(*it);
				return true;
			}else if(!actionFind){
				//else we remember this action in case there is no other possible action
				actionFind = true;
				res.state = "NEEDED"; //we return NEEDED in order to inform that the action is not possible
				res.action = ms->convertActionMStoAction(*it);
			}
		}
	}

	//Then we look for READY action
	vector<int> readyIds = db->getActionsIdFromState(req.agent, "READY");
	vector<supervisor_msgs::ActionMS> readyActions = ms->getActionsFromIds(readyIds);
	for(vector<supervisor_msgs::ActionMS>::iterator it = readyActions.begin(); it != readyActions.end(); it++){
		bool isActor = false;
		for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
			if(*ita == req.actor){
				isActor = true;
				break;
			}
		}
		if(isActor){//if we find an action READY to the actor, we return the action
			res.state = "READY";
			res.action = ms->convertActionMStoAction(*it);
			return true;
		}
	}

	//If we didn't find yet an action ASKED but not possible we look for NEEDED action
	if(!actionFind){
		vector<int> neededIds = db->getActionsIdFromState(req.agent, "NEEDED");
		vector<supervisor_msgs::ActionMS> neededActions = ms->getActionsFromIds(neededIds);
		for(vector<supervisor_msgs::ActionMS>::iterator it = neededActions.begin(); it != neededActions.end(); it++){
			bool isActor = false;
			for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
				if(*ita == req.actor){
					isActor = true;
					break;
				}
			}
			if(isActor){//if we find an action READY to the actor, we return the action
				res.state = "NEEDED";
				res.action = ms->convertActionMStoAction(*it);
				return true;
			}
		}
	}

	//if we find nothing we return the state NONE
	res.state = "NONE";
	return true;
}

/*
Service call to abort a goal for an agent
*/
bool getAllAgents(supervisor_msgs::GetAllAgents::Request  &req, supervisor_msgs::GetAllAgents::Response &res){
	
	res.agents =  db->getAgents();


	return true;
}

/*
Service call to get the state of an action
*/
bool getActionState(supervisor_msgs::GetActionState::Request  &req, supervisor_msgs::GetActionState::Response &res){
	
	pair<bool, supervisor_msgs::ActionMS> actionFind = ms->getActionFromAction(req.action);
	if(actionFind.first){
		res.state = db->getActionState(req.agent, actionFind.second);
	}else {
		res.state = "UNKNOWN";
	}


	return true;
}

/*
Service call to get the state of an action
*/
bool solveDivergentBelief(supervisor_msgs::SolveDivergentBelief::Request  &req, supervisor_msgs::SolveDivergentBelief::Response &res){
	
	pair<bool, supervisor_msgs::ActionMS> actionFind = ms->getActionFromAction(req.action);
	if(actionFind.first){
		string humanState = db->getActionState(req.agent, actionFind.second);
		string robotState = db->getActionState(robotName, actionFind.second);
		if(robotState == "READY" && humanState != "READY"){
			//we first look if the agent has the same plan as the robot
			pair<bool, supervisor_msgs::PlanMS> robotPlan = ms->getAgentPlan(robotName);
			if(robotPlan.first){
				pair<bool, supervisor_msgs::PlanMS> humanPlan = ms->getAgentPlan(req.agent);
				if(!humanPlan.first){
					if(simu){
						db->addActionsState(robotPlan.second.actions, req.agent, "PLANNED");
						db->addPlanState(robotPlan.second, req.agent, "PROGRESS");
					}else{
						//TODO: share plan
					}
					return true;
				}else{
					if(robotPlan.second.id != humanPlan.second.id){
						if(simu){
							db->addActionsState(robotPlan.second.actions, req.agent, "PLANNED");
							db->addPlanState(robotPlan.second, req.agent, "PROGRESS");
						}else{
							//TODO: share plan
						}
						return true;
					}
					//If the agent thinks the action is NEEDED, the problem comes from the preconditions
					if(humanState == "NEEDED"){
						for(vector<toaster_msgs::Fact>::iterator it = actionFind.second.prec.begin(); it != actionFind.second.prec.end(); it++){
							vector<toaster_msgs::Fact> toTest;
							toTest.push_back(*it);
							if(!db->factsAreIn(req.agent, toTest)){
								if(simu){
									db->addFacts(toTest, req.agent);
								}else{
									//TODO: give info
								}
							}	
						}
					}else if(humanState == "PLANNED"){//If the agent thinks the action is PLANNED, the problem comes from the previous actions
						//we get all actions id with a link to the action
						for(vector<supervisor_msgs::Link>::iterator itl = robotPlan.second.links.begin(); itl != robotPlan.second.links.end(); itl++){
							if(itl->following == actionFind.second.id){
								toaster_msgs::Fact fact;
								vector<toaster_msgs::Fact> toCheck;
								ostringstream toString;
								toString << itl->origin;
								fact.subjectId = toString.str();
								fact.property = "actionState";
								fact.targetId = "DONE";
								toCheck.push_back(fact);
								if(!db->factsAreIn(req.agent, toCheck)){
									if(simu){
									vector<supervisor_msgs::ActionMS> actions;
									actions.push_back(actionFind.second);
									db->addActionsState(actions, req.agent, "DONE");
									}else{
										//TODO: give action state
									}
								}
							}
						}
					}
				}
			}
		}else if(humanState == "READY" && robotState != "READY"){
			//TODO
		}
	}else {
		ROS_ERROR("[mental_states] No such action");
	}


	return true;
}


/*
Service call to know if a set of facts is on the mental state of an agent
*/
bool factsAreIn(supervisor_msgs::FactsAreIn::Request  &req, supervisor_msgs::FactsAreIn::Response &res){
	
	res.result = db->factsAreIn(req.agent, req.facts);

	return true;
}


/*
Service which return all knowledge of an agent
*/
bool getFactsAgents(supervisor_msgs::GetFactsAgent::Request  &req, supervisor_msgs::GetFactsAgent::Response &res){
	
	res.facts = db->getFactsAgent(req.agent);

	return true;
}


/*
Service which return all actions
*/
bool getActions(supervisor_msgs::GetActions::Request  &req, supervisor_msgs::GetActions::Response &res){
	
	res.actions = ms->getActionList();

	return true;
}


/*
Service which return all actions
*/
void update(MSManager* ms, string agent){
	
	 ms->update(agent);
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
  ros::ServiceServer service_abort_goal = node.advertiseService("mental_state/abort_goal", abortGoal); //abort a goal for an agent
  ros::ServiceServer service_info_given = node.advertiseService("mental_state/info_given", infoGiven); //when an agent informs an other agent about something
  ros::ServiceServer service_get_action_todo = node.advertiseService("mental_state/get_action_todo", getActionTodo); //get the action an agent thinks it has to do
  ros::ServiceServer service_get_all_agents = node.advertiseService("mental_state/get_all_agents", getAllAgents); //return all agents name
  ros::ServiceServer service_get_action_state = node.advertiseService("mental_state/get_action_state", getActionState); //return the state of an action in the knowledge
  ros::ServiceServer service_solve_divergent_belief = node.advertiseService("mental_state/solve_divergent_belief", solveDivergentBelief); //solve a divergent belief concerning an action
  ros::ServiceServer service_facts_are_in = node.advertiseService("mental_state/facts_are_in", factsAreIn); //say if a set of fact is in the mental state of an agent
  ros::ServiceServer service_get_facts_agents = node.advertiseService("mental_state/get_facts_agents", getFactsAgents); //return all knowledge of an agent
  ros::ServiceServer service_get_actions = node.advertiseService("mental_state/get_actions", getActions); //return all actions

  node.getParam("/simu", simu);
  node.getParam("/robot/name", robotName);
  allAgents = db->getAgents();
  ms->initGoals();
  ms->initHighLevelActions();

  ROS_INFO("[mental_state] mental_state ready");
  
  boost::thread_group g;
  while(true){
    for(vector<string>::iterator it = allAgents.begin(); it != allAgents.end(); it++){
	      g.create_thread(boost::bind(update, ms, *it));
    }
   g.join_all(); 
   ros::spinOnce();
  	loop_rate.sleep();
  }
  return 0;
}

/**
author Sandra Devin

Main class of the mental_state manager.

The mental state manager estimates and maintains the mental states of each agent concerning goals, plans and actions.

TODO: - weak achievement goal and plan
      - similar actions: ignore DONE and FAILED actions in actionState: changeActionStateFunction?

**/

#include <mental_states/ms_manager.h>

MSManager* ms = new MSManager();
string robotName;
vector<string> allAgents;
bool simu;


/*
Function call when we ask the robot to execute a new goal
*/
bool newGoal(string goal){
	
    supervisor_msgs::GoalMS* goalMS = NULL;
	//Get the goal by its name
    goalMS = ms->getGoalByName(goal);
    if(!goalMS){
		ROS_ERROR("[mental_state] Unknown goal");
		return false;
	}

	//add the goal state to PENDING into the knowledge of each of its actor
    vector<string> actors = goalMS->actors;
	for(vector<string>::iterator it = actors.begin(); it != actors.end(); it++){
        ms->db_.addGoalState(*goalMS, *it, "PENDING");
	}
	return true;
	
}


/*
Function call when we the robot starts to execute a goal
*/
bool startGoal(string goal){

    supervisor_msgs::GoalMS* goalMS = NULL;
    //Get the goal by its name
    goalMS = ms->getGoalByName(goal);
    if(!goalMS){
        ROS_ERROR("[mental_state] Unknown goal");
        return false;
    }

    //add the goal state to PROGRESS into the knowledge of each of its actor
    vector<string> actors = goalMS->actors;
    for(vector<string>::iterator it = actors.begin(); it != actors.end(); it++){
        ms->db_.addGoalState(*goalMS, *it, "PROGRESS");
    }
    return true;
}

/*
Function call to abort a goal for an agent
*/
bool abortGoal(string goal, string agent){
    supervisor_msgs::GoalMS* goalMS = NULL;
    //Get the goal by its name
    goalMS = ms->getGoalByName(goal);
    if(!goalMS){
        ROS_ERROR("[mental_state] Unknown goal");
        return true;
    }

    ms->db_.addGoalState(*goalMS, agent, "ABORTED");


    return true;
}

/*
Function call when a plan is computed by the robot
*/
bool newPlan(supervisor_msgs::Plan plan){

    //Transform the plan into a PlanMS
    supervisor_msgs::PlanMS planMS;
    planMS.goal = *(ms->getGoalByName(plan.goal));
    planMS.id = ms->getAndIncreasePlanId();
    map<int, int> lmap;
    for(vector<supervisor_msgs::Action>::iterator it = plan.actions.begin(); it != plan.actions.end(); it++){
        supervisor_msgs::ActionMS action = ms->createActionFromHighLevel(*it);
        planMS.actions.push_back(action);
        lmap[it->id] = action.id;
    }
    for(vector<supervisor_msgs::Link>::iterator it = plan.links.begin(); it != plan.links.end(); it++){
        supervisor_msgs::Link newLink;
        newLink.origin = lmap[it->origin];
        newLink.following = lmap[it->following];
        planMS.links.push_back(newLink);
    }

    //Add the plan to the list of plans
    ms->addPlanToList(planMS);

    //add the plan state to PROGRESS and the action to PLANNED into the robot knowledge and both to UNKNOWN into the other agent knowledge
    for(vector<string>::iterator it = allAgents.begin(); it != allAgents.end(); it++){
        if(*it == robotName){
            ms->db_.addActionsState(planMS.actions, *it, "PLANNED");
            ms->db_.addPlanState(planMS, *it, "PROGRESS");
        }else{
            ms->db_.addActionsState(planMS.actions, *it, "UNKNOWN");
            ms->db_.addPlanState(planMS, *it, "UNKNOWN");
        }
    }


    return true;
}

/*
Function call to abort the robot shares the current plan
*/
bool sharePlan(){

    //we get all the agents which can see the robot
    vector<string> presentAgents = ms->db_.getAgentsWhoSee(robotName);
    //We get the current plan
    pair<bool, supervisor_msgs::PlanMS> robotPlan = ms->getAgentPlan(robotName);
    if(!robotPlan.first){
        return true;
    }

    //For all these agents, the plan is now in PROGRESS and its actions PLANNED
    for(vector<string>::iterator it = presentAgents.begin(); it != presentAgents.end(); it++){
        ms->db_.addActionsState(robotPlan.second.actions, *it, "PLANNED");
        ms->db_.addPlanState(robotPlan.second, *it, "PROGRESS");
    }

    return true;
}

/*
Function call to abort the current plan for an agent
*/
bool abortPlan(string agent){

    ms->abortPlan(agent);


    return true;
}

/*
Function call when the robot detects the change of state of an action (either action in PROGRESS, DONE or ABORTED)
*/
bool actionState(supervisor_msgs::Action action, string state){

    //first we get the corresponding action
    pair<bool, supervisor_msgs::ActionMS> actionFind = ms->getActionFromAction(action);
    supervisor_msgs::ActionMS actionMS;
    if(actionFind.first){
        actionMS = actionFind.second;
    }else {
        actionMS = ms->createActionFromHighLevel(action);
    }

    //then we get all the agent which can see the actors of the action
    vector<string> canSeeAgents = allAgents;
    for(vector<string>::iterator it = actionMS.actors.begin(); it != actionMS.actors.end(); it++){
        vector<string> tempAgents = ms->db_.getAgentsWhoSee(*it);
        vector<string> temp(100);
        sort(tempAgents.begin(), tempAgents.end());
        sort(canSeeAgents.begin(), canSeeAgents.end());
        vector<string>::iterator i = set_intersection(tempAgents.begin(), tempAgents.end(), canSeeAgents.begin(), canSeeAgents.end(), temp.begin());
        temp.resize(i-temp.begin());
        canSeeAgents = temp;
    }
    canSeeAgents.insert(canSeeAgents.end(), actionMS.actors.begin(), actionMS.actors.end());
    canSeeAgents.push_back(robotName);

    //for all these agent and for the actors we change the state of the action
    for(vector<string>::iterator it = canSeeAgents.begin(); it != canSeeAgents.end(); it++){
        if(state == "DONE"){//if the state is done, we also add the effects of the action in the agent knowledge
            ms->db_.addEffects(actionMS.effects, *it);
        }
        vector<supervisor_msgs::ActionMS> actions;
        actions.push_back(actionMS);
        ms->db_.addActionsState(actions, *it, state);
    }

    return true;

}


/*
Service call when we change the state of a goal/plan/action
*/
bool changeState(supervisor_msgs::ChangeState::Request  &req, supervisor_msgs::ChangeState::Response &res){

    if(req.type == "action"){
        return actionState(req.action, req.state);
    }else if(req.type == "plan"){
        if(req.state == "PROGRESS"){
            return newPlan(req.plan);
        }else if(req.state == "SHARE"){
            return sharePlan();
        }else if(req.state == "ABORT"){
            return abortPlan(req.agent);
        }

    }else if(req.type == "goal"){
        if(req.state == "NEW"){
            return newGoal(req.goal);
        }else if(req.state == "PROGRESS"){
            return startGoal(req.goal);
        }else if(req.state == "ABORT"){
            return abortGoal(req.goal, req.agent);
        }

    }

}

/*
Function call to get the action an agent thinks an actor has to do
*/
pair<supervisor_msgs::Action, string> getActionTodo(string agent, string actor){

    bool actionFind = false;
    pair<supervisor_msgs::Action, string> answer;

    //First we look for the action ASKED to the actor in the knowledge of the agent
    vector<int> askedIds = ms->db_.getActionsIdFromState(agent, "ASKED");
    vector<supervisor_msgs::ActionMS> askedActions = ms->getActionsFromIds(askedIds);
    for(vector<supervisor_msgs::ActionMS>::iterator it = askedActions.begin(); it != askedActions.end(); it++){
        bool isActor = false;
        for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
            if(*ita == actor){
                isActor = true;
                break;
            }
        }
        if(isActor){//if we find an action ASKED to the actor, we look if the agent think the action is possible (preconditions)
            if(ms->db_.factsAreIn(agent, it->prec)){
                //if the action is possible we return this action
                answer.second = "READY"; //we return READY in order to inform that the action is possible
                answer.first = ms->convertActionMStoAction(*it);
                return answer;
            }else if(!actionFind){
                //else we remember this action in case there is no other possible action
                actionFind = true;
                answer.second = "NEEDED"; //we return NEEDED in order to inform that the action is not possible
                answer.first = ms->convertActionMStoAction(*it);
            }
        }
    }

    //Then we look for READY action
    vector<int> readyIds = ms->db_.getActionsIdFromState(agent, "READY");
    vector<supervisor_msgs::ActionMS> readyActions = ms->getActionsFromIds(readyIds);
    for(vector<supervisor_msgs::ActionMS>::iterator it = readyActions.begin(); it != readyActions.end(); it++){
        bool isActor = false;
        for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
            if(*ita == actor){
                isActor = true;
                break;
            }
        }
        if(isActor){//if we find an action READY to the actor, we return the action
            answer.second = "READY";
            answer.first = ms->convertActionMStoAction(*it);
            return answer;
        }
    }

    //If we didn't find yet an action ASKED but not possible we look for NEEDED action
    if(!actionFind){
        vector<int> neededIds = ms->db_.getActionsIdFromState(agent, "NEEDED");
        vector<supervisor_msgs::ActionMS> neededActions = ms->getActionsFromIds(neededIds);
        for(vector<supervisor_msgs::ActionMS>::iterator it = neededActions.begin(); it != neededActions.end(); it++){
            bool isActor = false;
            for(vector<string>::iterator ita = it->actors.begin(); ita != it->actors.end(); ita++){
                if(*ita == actor){
                    isActor = true;
                    break;
                }
            }
            if(isActor){//if we find an action READY to the actor, we return the action
                answer.second = "NEEDED";
                answer.first = ms->convertActionMStoAction(*it);
                return answer;
            }
        }
    }

    //if we find nothing we return the state NONE
    answer.second = "NONE";
    return answer;
}


/*
Function call to get the state of an action
*/
string getActionState(string agent, supervisor_msgs::Action action){

    pair<bool, supervisor_msgs::ActionMS> actionFind = ms->getActionFromAction(action);
    if(actionFind.first){
        return ms->db_.getActionState(agent, actionFind.second);
    }

    return "UNKNOWN";
}

/*
Service call when we want information
*/
bool getInfo(supervisor_msgs::GetInfo::Request  &req, supervisor_msgs::GetInfo::Response &res){

    if(req.info == "ALL_FACTS"){
        res.facts = ms->db_.getFactsAgent(req.agent);
    }else if(req.info == "FACTS_IN"){
        res.answer = ms->db_.factsAreIn(req.agent, req.facts);
    }else if(req.info == "AGENTS"){
        res.agents =  ms->db_.getAgents();
    }else if(req.info == "ACTIONS_TODO"){
        pair<supervisor_msgs::Action, string> answer = getActionTodo(req.agent, req.actor);
        res.action = answer.first;
        res.state = answer.second;
    }else if(req.info == "ACTION_STATE"){
        res.state = getActionState(req.agent, req.action);
    }else if(req.info == "ACTIONS"){
        res.actions = ms->getActionList();
    }

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
        ms->db_.addGoalState(*goal, req.receiver, req.state);
        ms->db_.addGoalState(*goal, req.sender, req.state);
	}else if(req.infoType == "planState"){//we put the state of the plan into the receiver and sender knowledge
		pair<bool, supervisor_msgs::PlanMS> plan = ms->getPlanFromId(req.planId);
		if(plan.first){
            ms->db_.addPlanState(plan.second, req.receiver, req.state);
            ms->db_.addPlanState(plan.second, req.sender, req.state);
		}
	}else if(req.infoType == "actionState"){//we put the state of the action into the receiver and sender knowledge
		pair<bool, supervisor_msgs::ActionMS> action = ms->getActionFromId(req.actionId);
		if(action.first){
			vector<supervisor_msgs::ActionMS> actions;
			actions.push_back(action.second);
            ms->db_.addActionsState(actions, req.receiver, req.state);
            ms->db_.addActionsState(actions, req.sender, req.state);
		}

	}else if(req.infoType == "fact"){//we put the fact into the receiver and sender knowledge
		vector<toaster_msgs::Fact> facts;
		facts.push_back(req.fact);
        ms->db_.addFacts(facts, req.receiver);
        ms->db_.addFacts(facts, req.sender);
	}else {
		ROS_ERROR("[mental_state] Unknown infoType");
		return false;
	}


	return true;
}



/*
Service call to get the state of an action
*/
bool solveDivergentBelief(supervisor_msgs::SolveDivergentBelief::Request  &req, supervisor_msgs::SolveDivergentBelief::Response &res){
	
	pair<bool, supervisor_msgs::ActionMS> actionFind = ms->getActionFromAction(req.action);
	if(actionFind.first){
        string humanState = ms->db_.getActionState(req.agent, actionFind.second);
        string robotState = ms->db_.getActionState(robotName, actionFind.second);
		if(robotState == "READY" && humanState != "READY"){
			//we first look if the agent has the same plan as the robot
			pair<bool, supervisor_msgs::PlanMS> robotPlan = ms->getAgentPlan(robotName);
			if(robotPlan.first){
				pair<bool, supervisor_msgs::PlanMS> humanPlan = ms->getAgentPlan(req.agent);
				if(!humanPlan.first){
					if(simu){
                        ms->db_.addActionsState(robotPlan.second.actions, req.agent, "PLANNED");
                        ms->db_.addPlanState(robotPlan.second, req.agent, "PROGRESS");
					}else{
						//TODO: share plan
					}
					return true;
				}else{
					if(robotPlan.second.id != humanPlan.second.id){
						if(simu){
                            ms->db_.addActionsState(robotPlan.second.actions, req.agent, "PLANNED");
                            ms->db_.addPlanState(robotPlan.second, req.agent, "PROGRESS");
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
                            if(!ms->db_.factsAreIn(req.agent, toTest)){
								if(simu){
                                    ms->db_.addFacts(toTest, req.agent);
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
                                if(!ms->db_.factsAreIn(req.agent, toCheck)){
									if(simu){
									vector<supervisor_msgs::ActionMS> actions;
									actions.push_back(actionFind.second);
                                    ms->db_.addActionsState(actions, req.agent, "DONE");
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
  ros::ServiceServer service_change_state = node.advertiseService("mental_state/change_state", changeState); //change the state of a goal/plan/action
  ros::ServiceServer service_get_info = node.advertiseService("mental_state/get_info", getInfo); //get diverse information
  ros::ServiceServer service_info_given = node.advertiseService("mental_state/info_given", infoGiven); //when an agent informs an other agent about something
  ros::ServiceServer service_solve_divergent_belief = node.advertiseService("mental_state/solve_divergent_belief", solveDivergentBelief); //solve a divergent belief concerning an action

  node.getParam("/simu", simu);
  node.getParam("/robot/name", robotName);
  allAgents = ms->db_.getAgents();
  ms->db_.cleanDB();
  ms->db_.initKnowledge(allAgents);
  ms->initGoals();
  ms->initHighLevelActions();

  ROS_INFO("[mental_state] mental_state ready");
  
  boost::thread_group g;
  while(node.ok()){
    ms->db_.updateKnowledge(); //we get new state from the database
    for(vector<string>::iterator it = allAgents.begin(); it != allAgents.end(); it++){
          g.create_thread(boost::bind(update, ms, *it));
    }
   g.join_all();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

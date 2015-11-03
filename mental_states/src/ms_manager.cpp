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
void MSManager::update(string agent){
	
 	//checkEffects();
        //computePreconditions();
        //planFeasibility(;
        //checkGoals();

}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "mental_state");

  ROS_INFO("Init mental_state");

  ros::NodeHandle node;

  ros::Rate loop_rate(30);

  DBInterface db;
  MSManager* ms;

  while (node.ok()) {

  vector<string> agents = db.getAgents();

  for(vector<string>::iterator it = agents.begin(); it != agents.end(); it++){
	ms->update(*it);
  }

  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}

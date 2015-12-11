/**
author Sandra Devin

Main class of the state_machines manager.

The state machines manager keeps trace of the activity of each agent.

**/

#include <state_machines/robot_sm.h>
#include <state_machines/human_sm.h>
#include "supervisor_msgs/GetAllAgents.h"

string robotState;
string humanState;



/*
Main function of the robot state machine
*/
void robotStateMachine(){
  	ros::Rate loop_rate(30);
	robotState = "IDLE";
	RobotSM rsm;

	
	while(true){
		if(robotState == "IDLE"){
			robotState = rsm.idleState();
		}else if(robotState == "ACTING"){
			robotState = rsm.actingState();
		}else if(robotState == "WAITING"){
			robotState = rsm.waitingState();
		}else{
			ROS_ERROR("[state_machines] Wrong robot state");	
		}
		ros::spinOnce();
  		loop_rate.sleep();
	}

}

/*
Main function of the human state machine
*/
void humanStateMachine(string human_name){
  	ros::Rate loop_rate(30);
	humanState = "IDLE";
	HumanSM* hsm = new HumanSM(human_name);
	
	while(true){
		if(humanState == "IDLE"){
			humanState = hsm->idleState();
		}else if(humanState == "ACTING"){
			humanState = hsm->actingState();
		}else if(humanState == "WAITING"){
			humanState = hsm->waitingState();
		}else if(humanState == "SHOULDACT"){
			humanState = hsm->shouldActState();
		}else if(humanState == "ABSENT"){
			humanState = hsm->shouldActState();
		}else{
			ROS_ERROR("[state_machines] Wrong human state");	
		}
		ros::spinOnce();
  		loop_rate.sleep();
	}

}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "state_machines");
  ros::NodeHandle node;

  ROS_INFO("[state_machines] Init state_machines");

  //Services declarations

  vector<string> all_agents;
  string robot_name;
  node.getParam("/robot/name", robot_name);
  ros::service::waitForService("mental_state/get_all_agents", -1);
  ros::ServiceClient client = node.serviceClient<supervisor_msgs::GetAllAgents>("mental_state/get_all_agents");
  supervisor_msgs::GetAllAgents srv;
  if (client.call(srv)){
	 all_agents = srv.response.agents;
  }else{
	ROS_ERROR("[state_machines] Failed to call service mental_state/get_all_agents");
  }

  ROS_INFO("[state_machines] state_machines ready");

  for(vector<string>::iterator it = all_agents.begin(); it != all_agents.end(); it++){
	if(*it == robot_name){
		boost::thread robotThread(robotStateMachine);
	}else{
  		boost::thread humanThread(boost::bind(humanStateMachine, *it));
	}
  }

  ros::spin();
  return 0;
}

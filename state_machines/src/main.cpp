/**
author Sandra Devin

Main class of the state_machines manager.

The state machines manager keeps trace of the activity of each agent.

**/

#include <state_machines/robot_sm.h>

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
			robotState = rsm.idleState(humanState);
		}else if(robotState == "ACTING"){
			robotState = rsm.actingState(humanState);
		}else if(robotState == "WAITING"){
			robotState = rsm.waitingState(humanState);
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
void humanStateMachine(){
  	ros::Rate loop_rate(30);
	humanState = "IDLE";
	
	while(true){
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

  ROS_INFO("[state_machines] state_machines ready");

  boost::thread robotThread(robotStateMachine);
  boost::thread humanThread(humanStateMachine);

  robotThread.join();
  humanThread.join();

  return 0;
}

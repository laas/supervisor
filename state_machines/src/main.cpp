/**
author Sandra Devin

Main class of the state_machines manager.

The state machines manager keeps trace of the activity of each agent.

**/

#include <state_machines/robot_sm.h>
#include <state_machines/human_sm.h>
#include "supervisor_msgs/GetInfo.h"
#include "supervisor_msgs/AgentActivity.h"
#include "supervisor_msgs/Focus.h"
#include "supervisor_msgs/HumanAction.h"

string robotState;
string robotName;
ros::NodeHandle* node;

string objectRobot_;
double weightRobot_ = 0.0;
bool stopableRobot_ = true;
bool actionPerformed = false;
supervisor_msgs::Action performedAction;
string agentAction;
ros::Publisher* tag_detection_pub_;

actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>* head_action_client;


void lookAtHuman(){


    //we get the coordonates of the object
    toaster_msgs::HumanListStamped humanList;
    double x,y,z;
   try{
       humanList  = *(ros::topic::waitForMessage<toaster_msgs::HumanListStamped>("pdg/humanList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Human>::iterator it = humanList.humanList.begin(); it != humanList.humanList.end(); it++){
         if(it->meAgent.meEntity.id == "HERAKLES_HUMAN1"){
             for(std::vector<toaster_msgs::Joint>::iterator itt = it->meAgent.skeletonJoint.begin(); itt != it->meAgent.skeletonJoint.end(); itt++){
                if(itt->meEntity.id == "head"){
                    x = itt->meEntity.pose.position.x;
                    y = itt->meEntity.pose.position.y;
                    z = itt->meEntity.pose.position.z;
                }
             }
            break;
         }
       }
   }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read human pose from toaster");
    }

    //we look at the object
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "map";
    goal.head_target_x = x;
    goal.head_target_y = y;
    goal.head_target_z = z;
    head_action_client->sendGoal(goal);



    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout){
        ROS_INFO("[action_executor] pr2motion head action did not finish before the time out.");
    }


}

/*
Look at an object
*/
void lookAt(string object){

    //we get the coordonates of the object
    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == object){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
   }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read %s pose from toaster", object.c_str());
    }

    //we look at the object
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "map";
    goal.head_target_x = x;
    goal.head_target_y = y;
    goal.head_target_z = z;
    head_action_client->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout){
        ROS_INFO("[action_executor] pr2motion head action did not finish before the time out.");
    }

    ros::Rate loop_rate(30);
    std_msgs::Bool msg;
    msg.data  = true;
    tag_detection_pub_->publish(msg);
    ros::Duration(0.05).sleep();
    msg.data  = false;
    tag_detection_pub_->publish(msg);
    ros::spinOnce();
}


/*
Main function of the robot state machine
*/
void robotStateMachine(){

  	ros::Rate loop_rate(30);
	robotState = "IDLE";
	RobotSM rsm;

    string topicName = "supervisor/activity_state/";
    topicName = topicName + robotName;
    ros::Publisher robot_pub = node->advertise<supervisor_msgs::AgentActivity>(topicName, 1000);


	while(true){
        supervisor_msgs::AgentActivity msg;
        string object;
        double weight = 0.0;
        bool stopable = true;
        string previousState = robotState;
		if(robotState == "IDLE"){
			robotState = rsm.idleState();
		}else if(robotState == "ACTING"){
            robotState = rsm.actingState();
            object = objectRobot_;
            weight = weightRobot_;
            stopable = stopableRobot_;
		}else if(robotState == "WAITING"){
			robotState = rsm.waitingState();
		}else{
			ROS_ERROR("[state_machines] Wrong robot state");	
        }
        if(robotState == "IDLE"){
            lookAtHuman();
        }

        //we publish the robot state
        msg.activityState = previousState;
        msg.unexpected = false;
        msg.importancy = weight;
        msg.object = object;
        msg.stopable = stopable;
        robot_pub.publish(msg);
        ros::spinOnce();
  		loop_rate.sleep();
    }

}

/*
Main function of the human state machine
*/
void humanStateMachine(string human_name){
  	ros::Rate loop_rate(30);
    string humanState = "IDLE";
    string topicName = "supervisor/activity_state/";
    topicName = topicName + human_name;
    ros::Publisher human_pub = node->advertise<supervisor_msgs::AgentActivity>(topicName, 1000);
	HumanSM* hsm = new HumanSM(human_name);

	
    while(true){
        //check if no action from the agent
        if(actionPerformed){
            if(agentAction == human_name){
                hsm->humanActs_ = true;
                hsm->PerformedAction_ = performedAction;
                actionPerformed = false;
                humanState == "ACTING";
            }
        }
        supervisor_msgs::AgentActivity msg;
        string object;
        double weight = 0.0;
        bool unexpected = false;
        string previousState = humanState;
		if(humanState == "IDLE"){
			humanState = hsm->idleState();
		}else if(humanState == "ACTING"){
            humanState = hsm->actingState(&object, &unexpected);
            weight = 0.8;
		}else if(humanState == "WAITING"){
			humanState = hsm->waitingState();
		}else if(humanState == "SHOULDACT"){
            humanState = hsm->shouldActState(robotState);
            weight = 0.8;
		}else if(humanState == "ABSENT"){
			humanState = hsm->absentState();
            unexpected = true;
		}else{
			ROS_ERROR("[state_machines] Wrong human state");	
        }
        if(previousState == "ACTING"){
            lookAt(object);
            if(robotState == "ACTING"){
                lookAt(objectRobot_);
            }
        }

        //we publish the robot state
        msg.activityState = previousState;
        msg.unexpected = unexpected;
        msg.importancy = weight;
        msg.object = object;
        human_pub.publish(msg);
        ros::spinOnce();
  		loop_rate.sleep();
	}

}

void focusCallback(const supervisor_msgs::Focus::ConstPtr& msg){

    objectRobot_ = msg->object;
    weightRobot_ = msg->weight;
    stopableRobot_ = msg->stopable;
}

/*
Service call to tell that a human performed an action
*/
bool humanAction(supervisor_msgs::HumanAction::Request  &req, supervisor_msgs::HumanAction::Response &res){

    actionPerformed = true;
    performedAction = req.action;
    agentAction = req.agent;

    return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "state_machines");
  ros::NodeHandle node_;
  node = &node_;
  ros::Rate loop_rate(30);

  ROS_INFO("[state_machines] Init state_machines");

  ros::ServiceServer service = node_.advertiseService("state_machines/human_action", humanAction);
  ros::Subscriber sub = node_.subscribe("action_executor/focus", 1000, focusCallback);
  ros::Publisher tag_detection_pub = node_.advertise <std_msgs::Bool>("ar_track_alvar/enable_detection",1);
  tag_detection_pub_ = &tag_detection_pub;

  while(tag_detection_pub_->getNumSubscribers() == 0)
      loop_rate.sleep();

  ROS_INFO("[state_machines] Waiting for head action server");
  head_action_client = new actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>("pr2motion/Head_Move_Target",true);
  head_action_client->waitForServer();

  vector<string> allAgents;
  node_.getParam("/robot/name", robotName);
  ros::service::waitForService("mental_state/get_info", -1);
  ros::ServiceClient client = node->serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
  supervisor_msgs::GetInfo srv;
  srv.request.info = "AGENTS";
  if (client.call(srv)){
	 allAgents = srv.response.agents;
  }else{
	ROS_ERROR("[state_machines] Failed to call service mental_state/get_all_agents");
  }

  std_msgs::Bool msg;
  msg.data  = false;
  tag_detection_pub.publish(msg);
  ros::spinOnce();

  ROS_INFO("[state_machines] state_machines ready");

  boost::thread_group g;
  for(vector<string>::iterator it = allAgents.begin(); it != allAgents.end(); it++){
    if(*it == robotName){
        g.create_thread(robotStateMachine);
    }else{
        g.create_thread(boost::bind(humanStateMachine, *it));
	}
  }

  ros::spin();
  return 0;
}

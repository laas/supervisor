/**
author Sandra Devin

Class wich contain connection to modules and information to keep from one action to another

**/

#include <action_executor/connector.h>

Connector::Connector(){

   node_.getParam("/simu", simu_);
   previousId_ = -1;

   //Init action clients
   acGTP_ = new actionlib::SimpleActionClient<gtp_ros_msg::requestAction>("gtp_ros_server", true);
   acGTP_->waitForServer();
   PR2motion_init_ = new actionlib::SimpleActionClient<pr2motion::InitAction>("pr2motion/Init", true);
   PR2motion_init_->waitForServer();
   PR2motion_torso_ = new actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>("pr2motion/Torso_Move", true);
   PR2motion_torso_->waitForServer();
   ROS_INFO("[action_executor] Action clients started.");
   
   //Init PR2motion
   ros::ServiceClient connect = node_.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
   pr2motion::InitGoal goal_init;
   PR2motion_init_->sendGoal(goal_init);
  
   pr2motion::connect_port srv;
   srv.request.local = "joint_state";
   srv.request.remote = "joint_states";
   if (!connect.call(srv)){
      ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
   }
   srv.request.local = "head_controller_state";
   srv.request.remote = "/head_traj_controller/state";
   if (!connect.call(srv)){
       ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
   }
   srv.request.local = "traj";
   srv.request.remote = "gtp_trajectory";
   if (!connect.call(srv)){
       ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
   }
   
   if(simu_){//change torso position
      pr2motion::Torso_MoveGoal goal;
      goal.torso_position = 0.1;
      PR2motion_torso_->sendGoal(goal);
      ROS_INFO("[action_executor] Waiting for Torso move");
      bool finishedBeforeTimeout = PR2motion_torso_->waitForResult(ros::Duration(waitActionServer_));

      if (!finishedBeforeTimeout){
         ROS_INFO("Action PR2Torso did not finish before the time out.");
      }
   }
}


/**
author Sandra Devin

Class wich contain connection to modules and information to keep from one action to another

**/

#include <action_executor/connector.h>

Connector::Connector(){

   node_.getParam("/simu", simu_);
   previousId_ = -1;
   idGrasp_ = -1;
   gripperRightOpen_ = false;
   gripperLeftOpen_ = false;
   rightArmMoving_ = false;
   leftArmMoving_ = false;
   rightGripperMoving_ = false;
   leftGripperMoving_ = false;
   torsoMoving_ = false;
   stopOrder_ = false;
   rightArmPose_ = "unknown";
   leftArmPose_ = "unknown";
   node_.getParam("/restPosition/right", rightArmRestPose_);
   node_.getParam("/restPosition/left", leftArmRestPose_);
   node_.getParam("/waitActionServer", waitActionServer_);
   node_.getParam("/shouldUseRightHand", shouldUseRightHand_);
   weightFocus_ = 0.0;

   //Init action clients
   acGTP_ = new actionlib::SimpleActionClient<gtp_ros_msg::requestAction>("gtp_ros_server", true);
   acGTP_->waitForServer();
   PR2motion_init_ = new actionlib::SimpleActionClient<pr2motion::InitAction>("pr2motion/Init", true);
   PR2motion_init_->waitForServer();
   PR2motion_torso_ = new actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>("pr2motion/Torso_Move", true);
   PR2motion_torso_->waitForServer();
   PR2motion_arm_right_ = new actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>("pr2motion/Arm_Right_Move",true);
   PR2motion_arm_right_->waitForServer();
   PR2motion_arm_left_ = new actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>("pr2motion/Arm_Left_Move",true);
   PR2motion_arm_left_->waitForServer();
   PR2motion_gripper_right_ = new actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>("pr2motion/Gripper_Right_Operate",true);
   PR2motion_gripper_right_->waitForServer();
   PR2motion_gripper_left_ = new actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>("pr2motion/Gripper_Left_Operate",true);
   PR2motion_gripper_left_->waitForServer();
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
      torsoMoving_ = true;
      PR2motion_torso_->sendGoal(goal);
      ROS_INFO("[action_executor] Waiting for Torso move");
      bool finishedBeforeTimeout = PR2motion_torso_->waitForResult(ros::Duration(waitActionServer_));
      torsoMoving_ = false;
      if (!finishedBeforeTimeout){
         ROS_INFO("Action PR2Torso did not finish before the time out.");
      }
   }
}


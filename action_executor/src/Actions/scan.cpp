/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/scan.h"
#include "action_executor/Actions/moveTo.h"

Scan::Scan(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
    if(action.parameters.size() == 1){
        object_ = action.parameters[0];
	}else{
        ROS_WARN("[action_executor] Wrong parameter numbers, should be: object");
    }
    connector->objectFocus_ = object_;
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = false;

    node_.getParam("/timeScan", timeScan_);
}

bool Scan::preconditions(){

   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
       ROS_WARN("[action_executor] The object to place is not a known manipulable object");
      return false;
   }
   
    return true;
}

bool Scan::plan(){

    return true;
}

bool Scan::exec(Server* action_server){

    //We go to rest pose if needed
    if(connector_->rightArmPose_ != connector_->rightArmRestPose_){
        supervisor_msgs::Action mvToAction;
        mvToAction.name = "moveTo";
        mvToAction.actors.push_back(robotName_);
        mvToAction.parameters.push_back(connector_->rightArmRestPose_);

        MoveTo mvToServer(mvToAction, connector_);
        if(mvToServer.preconditions()){
            if(mvToServer.plan()){
                if(mvToServer.exec(action_server)){
                    mvToServer.post();
                }
            }
        }

    }
    if(connector_->leftArmPose_ != connector_->leftArmRestPose_){
        supervisor_msgs::Action mvToAction;
        mvToAction.name = "moveTo";
        mvToAction.actors.push_back(robotName_);
        mvToAction.parameters.push_back(connector_->leftArmRestPose_);

        MoveTo mvToServer(mvToAction, connector_);
        if(mvToServer.preconditions()){
            if(mvToServer.plan()){
                if(mvToServer.exec(action_server)){
                    mvToServer.post();
                }
            }
        }

    }


    //TODO: add checking of head focus
    if(!connector_->simu_){
        controlRobotLight(true);
    }

    start_ = clock();
    double duration = 0.0;
    while(duration < timeScan_){
        if(action_server->isPreemptRequested() || connector_->stopOrder_){
            if(!connector_->simu_){
                controlRobotLight(false);
            }
            return false;
        }
        duration = (clock() - start_ ) / (double) CLOCKS_PER_SEC;
    }
    if(!connector_->simu_){
        controlRobotLight(false);
    }

    return true;

}

bool Scan::post(){

	return true;
}

void Scan::controlRobotLight(bool on){

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    int_param.name = "projector_mode";
    if(on){
        int_param.value = 3;
    }else{
        int_param.value = 1;
    }
    conf.ints.push_back(int_param);
    srv_req.config = conf;

    ros::service::call("/camera_synchronizer_node/set_parameters", srv_req, srv_resp);
}

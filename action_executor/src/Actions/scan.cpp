/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/scan.h"

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

    //TODO: add checking of head focus
    if(!connector_->simu_){
        //TODO: start red light
    }

    start_ = clock();
    double duration = 0.0;
    while(duration < timeScan_){
        if(action_server->isPreemptRequested() || connector_->stopOrder_){
            if(!connector_->simu_){
                //TODO: stop red light
            }
            return false;
        }
        duration = (clock() - start_ ) / (double) CLOCKS_PER_SEC;
    }
    if(!connector_->simu_){
        //TODO: stop red light
    }

    return true;

}

bool Scan::post(){

	return true;
}

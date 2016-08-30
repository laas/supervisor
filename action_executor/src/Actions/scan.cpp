/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/scan.h"
#include "action_executor/Actions/moveTo.h"

void Scan::isLookingCallback(const toaster_msgs::FactList::ConstPtr& msg){

    vector<toaster_msgs::Fact> agentMonitorFacts = msg->factList;
	bool found = false;
    for(vector<toaster_msgs::Fact>::iterator it = agentMonitorFacts.begin(); it != agentMonitorFacts.end(); it++){
        if(it->property == "IsLookingToward" && it->subjectId == robotToaster_ && it->targetId == object_){
            found = true;
            isLookingObject_ = true;
            break;
        }
    }
    if(!found){
		isLookingObject_ = false;
    }
}

Scan::Scan(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
    if(action.parameters.size() == 1){
        object_ = action.parameters[0];
	}else{
        ROS_WARN("[action_executor] Wrong parameter numbers, should be: object");
    }
    connector->objectFocus_ = object_;
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = false;
    originalAction_ = action;
    
    isLookingObject_ =false;

    node_.getParam("/timeScan", timeScan_);
    node_.getParam("/timeWaitScan", timeWait_);
    node_.getParam("robot/toasterName", robotToaster_);
    
    subLook_ = node_.subscribe("agent_monitor/factList", 1000, &Scan::isLookingCallback, this);
}

bool Scan::preconditions(){

   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
       ROS_WARN("[action_executor] The object to place is not a known manipulable object");
      return false;
   }

   //If the object is not refined, we refine it
   if(!objectRefined_){
      string refinedObject = refineObject(object_);
      if(refinedObject == "NULL"){
          ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
          return false;
      }else{
           object_ = refinedObject;
      }
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


    //Checking of head focus
    bool timerWaitStarted = false;
    double durationWait = 0.0;
    while(!isLookingObject_ || durationWait < timeWait_){
		if(timerWaitStarted){
			wait_ = clock();
		}else{
			durationWait = (clock() - wait_) / (double) CLOCKS_PER_SEC;
			timerWaitStarted = true;
		}
	}
	//we failed to lok the object
	if(!isLookingObject_){
		ROS_WARN("[action_executor] the scan action failed because the head was not looking the object");
		return false;
	}
    
    if(!connector_->simu_){
        controlRobotLight(true);
    }

    start_ = clock();
    double duration = 0.0;
    timerWaitStarted = false;
    durationWait = 0.0;
    bool isScanning = true;
    bool shouldScan = true;
    while(shouldScan){
        if(action_server->isPreemptRequested() || connector_->stopOrder_){
            if(!connector_->simu_){
                controlRobotLight(false);
            }
            isScanning = false;
            return false;
        }else if(!isLookingObject_){
			if(!connector_->simu_){
				controlRobotLight(false);
			}
			isScanning = false;
			if(timerWaitStarted){
				wait_ = clock();
			}else{
				durationWait = (clock() - wait_) / (double) CLOCKS_PER_SEC;
				if(durationWait >= timeWait_){
					ROS_WARN("[action_executor] the scan action failed dur to head looking interruption");
					return false;
				}
				timerWaitStarted = true;
			}
		}else{
			if(isScanning){
				duration = (clock() - start_ ) / (double) CLOCKS_PER_SEC;
				if(duration >= timeScan_){
					shouldScan = false;
				}
			}else{
				if(!connector_->simu_){
					controlRobotLight(true);
				}
				duration = 0.0;
				start_ = clock();
				isScanning = true;
			}
		}
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

supervisor_msgs::Action Scan::getInstantiatedAction(){

    supervisor_msgs::Action action = originalAction_;

    vector<string> newParams;
    newParams.push_back(object_);
    action.parameters = newParams;

    return action;
}

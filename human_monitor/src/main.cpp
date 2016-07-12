/**
author Sandra Devin

Main class of the human_monitor.

The human monitor allows to monitor humans actions by looking into facts from toaster

**/

#include <human_monitor/human_monitor.h>

HumanMonitor* hm = new HumanMonitor();
string humanHand;
double pickThreshold, placeThreshold, dropThreshold;

/*
Service call from simulation to tell that a human picked an object
*/
bool humaActionSimu(supervisor_msgs::HumanActionSimu::Request  &req, supervisor_msgs::HumanActionSimu::Response &res){
	
	if(req.actionName == "pick"){
	   hm->humanPick(req.agent, req.object);
	}else if(req.actionName == "place"){
	   hm->humanPlace(req.agent, req.object, req.support);
	}else if(req.actionName == "drop"){
	   hm->humanDrop(req.agent, req.object, req.container);
	}else{
	   ROS_ERROR("[human_monitor] Unknown action name");
	}

	return true;
}


/*
Call back for the topic agent_monitor/fact_list
*/
void agentFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){
	
    vector<toaster_msgs::Fact> facts = msg->factList;

    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "Distance" && it->subjectId == humanHand){
            pair<bool, string> ownerAttachment = hm->hasInHand(it->subjectOwnerId);
            if(ownerAttachment.first){
                if(it->targetId != ownerAttachment.second){
                    if(it->doubleValue < placeThreshold && hm->isSupportObject(it->targetId)){
                        hm->humanPlace(it->subjectOwnerId, ownerAttachment.second, it->targetId);
                    }else if(it->doubleValue < dropThreshold && hm->isContainerObject(it->targetId)){
                            hm->humanDrop(it->subjectOwnerId, ownerAttachment.second, it->targetId);
                    }
                }
            }else{
                if(it->doubleValue < pickThreshold && hm->isManipulableObject(it->targetId)){
                    hm->humanPick(it->subjectOwnerId, it->targetId);
                }
            }
        }
    }
	
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_monitor");
  ros::NodeHandle node;
  node.getParam("/humanRightHand", humanHand);
  node.getParam("/threshold/pick", pickThreshold);
  node.getParam("/threshold/place", placeThreshold);
  node.getParam("/threshold/drop", dropThreshold);

  ROS_INFO("[human_monitor] Init human_monitor");

  //Services declarations
  ros::ServiceServer service_action = node.advertiseService("human_monitor/human_action_simu", humaActionSimu); //allows the simulation to tell that a human has done an action
  
  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1000, agentFactListCallback);

  ROS_INFO("[human_monitor] human_monitor ready");

  ros::spin();

  return 0;
}

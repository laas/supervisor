/**
 *\author Sandra Devin (sdevin@laas.fr)
 *
 * @todo add inference of actions based on effects of known needed actions
 *
 * **********/


#include <human_monitor/human_monitor.h>

ros::NodeHandle* node_;
HumanMonitor* hm_;
std::string humanHand;
double pickThreshold, placeThreshold, dropThreshold, disengageThreshold;
std::map<std::string, std::string> humanEngaged;
bool shouldDetect;

/**
* \brief Service call from simulation to tell that a human does an action
* @param req service request
* @param res service result
* @return true
* */
bool humaActionSimu(supervisor_msgs::HumanAction::Request  &req, supervisor_msgs::HumanAction::Response &res){

    //we look for actions parameters
    if(req.action.parameter_keys.size() != req.action.parameter_values.size()){
         ROS_ERROR("[human_monitor] Invalid action parameters: nb keys should be equal to nb values!");
         return true;
    }
    std::map<std::string, std::string> params;
    for(int i = 0; i < req.action.parameter_keys.size(); i++){
         if(req.action.parameter_keys[i] == "object"){
             params["object"] = req.action.parameter_values[i];
         }
         if(req.action.parameter_keys[i] == "support"){
             params["support"] = req.action.parameter_values[i];
         }
         if(req.action.parameter_keys[i] == "container"){
             params["container"] = req.action.parameter_values[i];
         }
         if(req.action.parameter_keys[i] == "support1"){
             params["support1"] = req.action.parameter_values[i];
         }
         if(req.action.parameter_keys[i] == "support2"){
             params["support2"] = req.action.parameter_values[i];
         }
    }

    //we send the action
    if(req.action.name == "pick"){
        if(params.find("object") == params.end()){
            ROS_ERROR("[human_monitor] No object to pick!");
            return true;
        }
       hm_->humanPick(req.agent, params["object"]);
    }else if(req.action.name == "place"){
        if(params.find("object") == params.end()){
            ROS_ERROR("[human_monitor] No object to place!");
            return true;
        }
        if(params.find("support") == params.end()){
            ROS_ERROR("[human_monitor] No support to place!");
            return true;
        }
       hm_->humanPlace(req.agent, params["object"], params["support"]);
    }else if(req.action.name == "drop"){
        if(params.find("object") == params.end()){
            ROS_ERROR("[human_monitor] No object to drop!");
            return true;
        }
        if(params.find("container") == params.end()){
            ROS_ERROR("[human_monitor] No container to drop!");
            return true;
        }
       hm_->humanDrop(req.agent, params["object"], params["container"]);
    }else if(req.action.name == "placeStick"){
        if(params.find("object") == params.end()){
            ROS_ERROR("[human_monitor] No object to place!");
            return true;
        }
        if(params.find("support1") == params.end() || params.find("support2") == params.end()){
            ROS_ERROR("[human_monitor] No support to place!");
            return true;
        }
       hm_->humanPlaceStick(req.agent, params["object"], params["support1"], params["support2"]);
    }else{
       ROS_ERROR("[human_monitor] Unknown action name");
    }

    return true;
}

/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
/*
*/
void agentFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){

    if(shouldDetect){
        std::vector<toaster_msgs::Fact> facts = msg->factList;

        for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            if(it->property == "Distance" && it->subjectId == humanHand){
                if(humanEngaged.find(it->subjectOwnerId) != humanEngaged.end()){
                    //the human just performed an action, we wait he disengages
                    if(it->targetId == humanEngaged[it->subjectOwnerId] && it->doubleValue > disengageThreshold){
                            humanEngaged.erase(it->subjectOwnerId);
                    }
                }else{
                    std::pair<bool, std::string> ownerAttachment = hm_->hasInHand(it->subjectOwnerId);
                    if(ownerAttachment.first){
                        //the human has an object in hand, we look for place or drop
                        if(it->targetId != ownerAttachment.second){
                          if(it->doubleValue < placeThreshold && hm_->isSupportObject(it->targetId)){
                            hm_->humanPlace(it->subjectOwnerId, ownerAttachment.second, it->targetId);
                            humanEngaged[it->subjectOwnerId] = it->targetId;
                          }else if(it->doubleValue < dropThreshold && hm_->isContainerObject(it->targetId)){
                                hm_->humanDrop(it->subjectOwnerId, ownerAttachment.second, it->targetId);
                                humanEngaged[it->subjectOwnerId] = it->targetId;
                          }
                        }
                    }else{
                        //the human has no object in hand, we look for a pick
                        if(it->doubleValue < pickThreshold && hm_->isManipulableObject(it->targetId)){
                            hm_->humanPick(it->subjectOwnerId, it->targetId);
                            humanEngaged[it->subjectOwnerId] = it->targetId;
                        }
                    }
                }
            }
        }
    }

}
/**
 * \brief Main function
 * */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_monitor");
  ros::NodeHandle node;
  node_ = &node;


  ROS_INFO("[human_monitor] Init");

  node.getParam("/human_monitor/rightHand", humanHand);
  node.getParam("/human_monitor/threshold/pick", pickThreshold);
  node.getParam("/human_monitor/threshold/place", placeThreshold);
  node.getParam("/human_monitor/threshold/drop", dropThreshold);
  node.getParam("/human_monitor/threshold/disengage", disengageThreshold);
  node.getParam("/human_monitor/shouldDetect", shouldDetect);

  HumanMonitor hm(node_);
  hm_ = &hm;


  ros::ServiceServer service_action = node.advertiseService("human_monitor/human_action_simu", humaActionSimu); //allows the simulation to tell that a human has done an action

  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1, agentFactListCallback);

  ROS_INFO("[human_monitor] Ready");

  ros::spin();

}

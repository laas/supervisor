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
std::string robotName_;
std::vector<std::string> humanStack_;
std::string topStack_, onScan1_, onScan2_, humanTape_;
std::vector<supervisor_msgs::Action> previousActions_;
std::map<std::string, std::string> colors_;

/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanPickStack(std::string agent){

    hm_->humanPick(agent, topStack_);
    humanEngaged[agent] = topStack_;
    if(humanStack_.size() > 0){
        topStack_ = humanStack_.front();
        humanStack_.erase(humanStack_.begin());
    }else{
        topStack_ = "NULL";
    }
}

/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanPickTape(std::string agent){

    hm_->humanPick(agent, humanTape_);
    humanEngaged[agent] = humanTape_;
    humanTape_ = "NULL";
}


/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanPickArea1(std::string agent){

    hm_->humanPick(agent, onScan1_);
    humanEngaged[agent] = onScan1_;
    onScan1_ = "NULL";
}

/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanPickArea2(std::string agent){

    hm_->humanPick(agent, onScan2_);
    humanEngaged[agent] = onScan2_;
    onScan2_ = "NULL";
}


/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanPlaceArea1(std::string agent, std::string object){

    onScan1_ = object;
    hm_->humanPlace(agent, object, "SCAN_AREA1");
    humanEngaged[agent] = "SCAN_AREA1";
}


/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanPlaceArea2(std::string agent, std::string object){

    onScan2_ = object;
    hm_->humanPlace(agent, object, "SCAN_AREA2");
    humanEngaged[agent] = "SCAN_AREA2";
}


/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanDropGreen(std::string agent, std::string object){

    hm_->humanDrop(agent, object, "GREEN_BOX");
    humanEngaged[agent] = "GREEN_BOX";
}

/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
void humanDropRed(std::string agent, std::string object){

    hm_->humanDrop(agent, object, "RED_BOX2");
    humanEngaged[agent] = "RED_BOX2";
}

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
         if(req.action.parameter_keys[i] == "position"){
             params["position"] = req.action.parameter_values[i];
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
    }else if(req.action.name == "goTo"){
        if(params.find("position") == params.end()){
            ROS_ERROR("[human_monitor] No position where to go!");
            return true;
        }
       hm_->humanGoTo(req.agent, params["position"]);
    }else if(req.action.name == "pickStack"){
       humanPickStack(req.agent);
    }else if(req.action.name == "pickTape"){
        humanPickTape(req.agent);
     }else if(req.action.name == "pickArea1"){
        humanPickArea1(req.agent);
     }else if(req.action.name == "pickArea2"){
        humanPickArea2(req.agent);
     }else if(req.action.name == "placeArea1"){
        std::pair<bool, std::string> ownerAttachment = hm_->hasInHand(req.agent);
        if(ownerAttachment.first){
            humanPlaceArea1(req.agent, ownerAttachment.second);
        }else{
            ROS_ERROR("[human_monitor] No object in human hand!");
        }
     }else if(req.action.name == "placeArea2"){
        std::pair<bool, std::string> ownerAttachment = hm_->hasInHand(req.agent);
        if(ownerAttachment.first){
            humanPlaceArea2(req.agent, ownerAttachment.second);
        }else{
            ROS_ERROR("[human_monitor] No object in human hand!");
        }
     }else if(req.action.name == "dropGreen"){
        std::pair<bool, std::string> ownerAttachment = hm_->hasInHand(req.agent);
        if(ownerAttachment.first){
            humanDropGreen(req.agent, ownerAttachment.second);
        }else{
            ROS_ERROR("[human_monitor] No object in human hand!");
        }
     }else if(req.action.name == "dropRed"){
        std::pair<bool, std::string> ownerAttachment = hm_->hasInHand(req.agent);
        if(ownerAttachment.first){
            humanDropRed(req.agent, ownerAttachment.second);
        }else{
            ROS_ERROR("[human_monitor] No object in human hand!");
        }
     }else{
       ROS_ERROR("[human_monitor] Unknown action name");
    }

    return true;
}


/**
* \brief Call back for the topic agent_monitor/fact_list
* @param msg agent monitor fact list
* */
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
                          if(it->doubleValue < placeThreshold && it->targetId == "SCAN_AREA1"){
                            humanPlaceArea1(it->subjectOwnerId, ownerAttachment.second);
                          }
                          if(it->doubleValue < placeThreshold && it->targetId == "SCAN_AREA2"){
                            humanPlaceArea2(it->subjectOwnerId, ownerAttachment.second);
                          }
                          if(it->doubleValue < dropThreshold && colors_[ownerAttachment.second] == "green" && it->targetId == "GREEN_BOX"){
                                humanDropGreen(it->subjectOwnerId, ownerAttachment.second);
                          }
                          if(it->doubleValue < dropThreshold && colors_[ownerAttachment.second] == "red" && it->targetId == "RED_BOX2"){
                                humanDropRed(it->subjectOwnerId, ownerAttachment.second);
                          }
                        }
                    }else{
                        //the human has no object in hand, we look for a pick
                        if(it->doubleValue < pickThreshold && topStack_ == it->targetId){
                            humanPickStack(it->subjectOwnerId);
                        }
                        if(it->doubleValue < pickThreshold && onScan1_ == it->targetId){
                            humanPickArea1(it->subjectOwnerId);
                        }
                        if(it->doubleValue < pickThreshold && onScan2_ == it->targetId){
                            humanPickArea2(it->subjectOwnerId);
                        }
                        if(it->doubleValue < pickThreshold && humanTape_ == it->targetId){
                            humanPickTape(it->subjectOwnerId);
                        }
                    }
                }
            }
        }
    }

}

/**
 * \brief Callback of the previous action topic
 * @param msg topic msg
 * */
void previousActionCallback(const supervisor_msgs::ActionsList::ConstPtr& msg){

    std::vector<supervisor_msgs::Action> newPrev = msg->actions;
    if(newPrev.size() > previousActions_.size()){
        for(int i = previousActions_.size(); i < newPrev.size(); i++){
            if(newPrev[i].actors[0] == robotName_){
                if(newPrev[i].name == "pickandplace"){
                    //get object and support
                    std::string object, support;
                    for(int j = 0; j < newPrev[i].parameter_keys.size(); j++){
                        if(newPrev[i].parameter_keys[j] == "object"){
                            object = newPrev[i].parameter_values[j];
                        }
                        if(newPrev[i].parameter_keys[j] == "support"){
                            support = newPrev[i].parameter_values[j];
                        }
                    }
                    if(support == "SCAN_AREA1"){
                        onScan1_ = object;
                    }
                    if(support == "SCAN_AREA2"){
                        onScan2_ = object;
                    }
                }
            }
        }
        previousActions_ = newPrev;
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
  node.getParam("/supervisor/robot/name", robotName_);
  node.getParam("/human_monitor/threshold/pick", pickThreshold);
  node.getParam("/human_monitor/threshold/place", placeThreshold);
  node.getParam("/human_monitor/threshold/drop", dropThreshold);
  node.getParam("/human_monitor/threshold/disengage", disengageThreshold);
  node.getParam("/human_monitor/shouldDetect", shouldDetect);
  node.getParam("/human_monitor/humanStack", humanStack_);

  HumanMonitor hm(node_);
  hm_ = &hm;

  if(humanStack_.size() > 0){
      topStack_ = humanStack_.front();
      humanStack_.erase(humanStack_.begin());
  }else{
      topStack_ = "NULL";
  }
  onScan1_ = "NULL";
  onScan2_ = "NULL";
  humanTape_ = "RED_TAPE2";
  colors_["GREEN_CUBE1"] = "green";
  colors_["GREEN_CUBE2"] = "green";
  colors_["GREEN_CUBE3"] = "green";
  colors_["RED_CUBE1"] = "red";
  colors_["RED_CUBE2"] = "red";
  colors_["RED_CUBE3"] = "red";


  ros::ServiceServer service_action = node.advertiseService("human_monitor/human_action_simu", humaActionSimu); //allows the simulation to tell that a human has done an action

  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1, agentFactListCallback);
  ros::Subscriber sub_prev_action = node.subscribe("/supervisor/previous_actions", 1, previousActionCallback);

  ROS_INFO("[human_monitor] Ready");

  ros::spin();

}

/**
author Sandra Devin
Class allowing the execution of a place action
**/

#include "action_executor/Actions/place.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
Place::Place(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    bool foundObj = false;
    bool foundSup = false;
    for(int i=0; i<=action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "object"){
            object_ = action.parameter_values[i];
            foundObj = true;
        }
        if(action.parameter_keys[i] == "support"){
            support_ = action.parameter_values[i];
            foundSup = true;
        }
        if(foundObj && foundSup){
            break;
        }
    }
    if(!foundObj){
        ROS_WARN("[action_executor] Missing parameter: object to palce");
    }
    if(!foundSup){
        ROS_WARN("[action_executor] Missing parameter: support where to palce");
    }

    replacementSupport_ = "NONE";
}

/**
 * \brief Precondition of the template action:
 *    - describe here the preconditions
 * @return true if the preconditions are checked
 * */
bool Place::preconditions(){

    //First we check if the object is a known manipulable object
    if(!isManipulableObject(object_)){
       ROS_WARN("[action_executor] The object to place is not a known manipulable object");
      return false;
    }

    //Then we check if the support is a known support object
    if(!isSupportObject(support_)){
       ROS_WARN("[action_executor] The support is not a known support object");
      return false;
    }

    //Then we check if the robot has the object in hand and if the support is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = support_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the template action:
 *    - check if stack or place action
 *    - check if other placement to use for planning
 *    - check if there is a required point associated with the object/support
 *    - ask a plan to gtp
 * @return true if the planning succeed
 * */
bool Place::plan(){

    //If there is no previous task we look for a previous grasp
    if(connector_->previousId_ == -1){
        if(connector_->idGrasp_ == -1){
            ROS_WARN("[action_executor] No previous Id nore previous grasp id");
           return false;
        }else{
            //TODO: add attachment in GTP
        }
    }

    std::vector<gtp_ros_msgs::Role> agents;
    gtp_ros_msgs::Role role;
    role.role = "mainAgent";
    role.name = connector_->robotName_;
    agents.push_back(role);
    std::vector<gtp_ros_msgs::Role> objects;
    role.role = "mainObject";
    role.name = object_;
    objects.push_back(role);
    std::vector<gtp_ros_msgs::Point> points;
    std::vector<gtp_ros_msgs::MiscData> datas;

    if(connector_->shouldUseRightHand_){
        gtp_ros_msgs::MiscData data;
        data.key = "hand";
        data.value = "right";
        datas.push_back(data);
    }

    std::string actionName;
    if(isManipulableObject(support_)){
        role.role = "supportObject";
        role.name = support_;
        objects.push_back(role);
        //if the support is also a manipulable object, this is a stack action
        actionName = "stackObj";
        //we add a point in order the objects to be align
        gtp_ros_msgs::Point point;
        point.key = "target";
        point.point.x = 0.0;
        point.point.y = 0.0;
        point.point.z = 0.0;
        points.push_back(point);
    }else{
        actionName = "place";
        std::string replacementTopic = "/action_executor/planningSupport/" + support_;
        //we look if there is a more specific placement to use for planning
        if(connector_->node_->hasParam(replacementTopic)){
            connector_->node_->getParam(replacementTopic, replacementSupport_);
            role.role = "supportObject";
            role.name = replacementSupport_;
            objects.push_back(role);
        }else{
            role.role = "supportObject";
            role.name = support_;
            objects.push_back(role);
            //we look if there is a specific point where to place the object
            std::string xParamTopic = "/action_executor/points/"+ support_ + "/" + object_ + "/x";
            std::string yParamTopic = "/action_executor/points/"+ support_ + "/" + object_ + "/y";
            std::string thetaParamTopic = "/action_executor/points/"+ support_ + "/" + object_ + "/theta";
            if(connector_->node_->hasParam(xParamTopic)){
                double pointX, pointY, pointTheta;
                connector_->node_->getParam(xParamTopic, pointX);
                connector_->node_->getParam(yParamTopic, pointY);
                connector_->node_->getParam(thetaParamTopic, pointTheta);
                double x, y;
                try{
                    //we look for the suppport position
                    toaster_msgs::ObjectListStamped objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
                    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
                      if(it->meEntity.id == support_){
                         x = it->meEntity.pose.position.x;
                         y = it->meEntity.pose.position.y;
                         break;
                      }
                    }
                    x = x + pointX;
                    y = y + pointY;
                    gtp_ros_msgs::Point point;
                    point.key = "target";
                    point.point.x = x;
                    point.point.y = y;
                    point.point.z = pointTheta;
                    points.push_back(point);
                }
                catch(const std::exception & e){
                    ROS_WARN("[action_executor] Failed to read %s pose from toaster", support_.c_str());
                }
           }
        }
    }
    std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP(actionName, agents, objects, datas, points);
    gtpActionId_ = answer.first;

    if(gtpActionId_ == -1){
        return false;
     }

    subSolutions_ = answer.second;
    return true;
}

/**
 * \brief Execution of the template action:
 *    - execute the gtp plan
 * @return true if the execution succeed
 * */
bool Place::exec(Server* action_server){

   return execAction(gtpActionId_, subSolutions_, false, action_server);

}

/**
 * \brief Post conditions of the template action:
 *    - put the object on the support
 * @return true if the post-conditions are ok
 * */
bool Place::post(){

    if(replacementSupport_ != "NONE"){
        PutOnSupport(object_, replacementSupport_);
    }
    else{
        PutOnSupport(object_, support_);
    }

    return true;
}

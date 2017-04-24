/**
author Sandra Devin

Simple dialogue node

**/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "std_msgs/Bool.h"

#include "supervisor_msgs/GiveInfo.h"
#include "supervisor_msgs/Ask.h"
#include "supervisor_msgs/SharedPlan.h"
#include "supervisor_msgs/String.h"
#include "supervisor_msgs/Info.h"


#ifdef ACAPELA
#include "acapela/InitAction.h"
#include "acapela/SetVoice.h"
#include "acapela/SayAction.h"
#endif //ACAPELA

ros::NodeHandle* node_;
bool shoudlSpeak_;
std::string robotName_;
double timeToWait_;
std::vector<std::pair<std::string, std::pair<toaster_msgs::Fact, bool> > > toSayFacts;
std::vector<std::pair<std::string, std::pair<supervisor_msgs::Action, std::string> > > toSayActions;
std::vector<std::pair<std::string, std::pair<supervisor_msgs::SharedPlan, std::string> > > toSayPlans;
std::vector<std::pair<std::string, supervisor_msgs::SharedPlan> > toSharePlans;

ros::Publisher info_pub_;
ros::Publisher speaking_pub_;

#ifdef ACAPELA
double waitActionServer;
actionlib::SimpleActionClient<acapela::InitAction>* acInit;
actionlib::SimpleActionClient<acapela::SayAction>* acSay;
#endif //ACAPELA


/**
 * \brief Verbalize a sentence
 * @param sentence the sentence to say
 * @param receiver the person to talk to (not used for now)
 * */
void saySentence(std::string sentence, std::string receiver){

    //publish the intention to speak
    std_msgs::Bool msg;
    msg.data = true;
    speaking_pub_.publish(msg);

    ROS_INFO("[DIALOGUE] ROBOT: %s", sentence.c_str());

    #ifdef ACAPELA
    if(shoudlSpeak_){
        acapela::SayGoal goal;
        goal.message = sentence;
        acSay->sendGoal(goal);
        bool finishedBeforeTimeout = acSay->waitForResult(ros::Duration(waitActionServer));
        if (!finishedBeforeTimeout){
           ROS_INFO("Acapela say did not finish before the time out.");
        }
    }
    #endif //ACAPELA

    //publish the end of the speak
    msg.data = false;
    speaking_pub_.publish(msg);
}

/**
 * \brief Verbalize a fact
 * @param fact the fact to verbalize
 * @param isTrue true if the fact is true
 * @param receiver the person to talk to (not used for now)
 * */
void giveInfoFact(toaster_msgs::Fact fact, bool isTrue, std::string receiver){

    //We transform the fact into a sentence
    std::string sentence;
    std::string isReverseTopic, subjectTopic, targetTopic, propertyTopic;
    std::string subjectName, targetName, propertyName;
    bool isReverse;
    isReverseTopic = "dialogue_node/properties/isReverse/" + fact.property;
    node_->getParam(isReverseTopic, isReverse);
    if(fact.subjectId == robotName_){
        if(isReverse){
            subjectName = "I";
        }else{
            subjectName = "me";
        }
    }else if(fact.subjectId == receiver){
        subjectName = "you";
    }else{
        subjectTopic = "dialogue_node/entitiesTranslation/" + fact.subjectId;
        node_->getParam(subjectTopic, subjectName);
    }
    if(fact.targetId == robotName_){
        if(isReverse){
            targetName = "I";
        }else{
            targetName = "me";
        }
    }else if(fact.targetId == receiver){
        targetName = "you";
    }else{
        targetTopic = "dialogue_node/entitiesTranslation/" + fact.targetId;
        node_->getParam(targetTopic, targetName);
    }
    if(isTrue){
        propertyTopic = "dialogue_node/properties/translationTrue/" + fact.property;
    }else{
        propertyTopic = "dialogue_node/properties/translationFalse/" + fact.property;
    }
    node_->getParam(propertyTopic, propertyName);
    if(isReverse){
        sentence = targetName + " " + propertyName + " the " + subjectName;
    }else{
        sentence = "The " + subjectName + " " + propertyName + " the " + targetName;
    }

    //We verbalize the sentence
    saySentence(sentence, receiver);

    supervisor_msgs::Info msg;
    msg.toRobot = false;
    msg.agent = receiver;
    msg.type = "FACT";
    msg.fact = fact;
    msg.isTrue = isTrue;
    info_pub_.publish(msg);
}

/**
 * \brief Give the state of an action
 * @param action the concerned action
 * @param actionState the state of the action
 * @param receiver the person to talk to
 * */
void giveInfoAction(supervisor_msgs::Action action, std::string actionState, std::string receiver){

    /** @todo: find a better way to do this */
    std::string sentence;
    if(action.name == "pick"){
        std::string objectTopic, actorTopic;
        std::string objectName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " picked the " + objectName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " can not pick the " + objectName;
        }else if(actionState == "NOT_PERFORMED"){
            sentence = actorName + " did not pick the " + objectName;
        }else if(actionState == "WILL"){
            sentence = actorName + " will pick the " + objectName;
        }else if(actionState == "SHOULD"){
            sentence = actorName + " should pick the " + objectName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "place" || action.name == "pickandplace"){
        std::string objectTopic, supportTopic, actorTopic;
        std::string objectName, supportName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for place action");
            return;
        }
        bool supportFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "support"){
                supportTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(supportTopic, supportName);
                supportFound = true;
            }
        }
        if(!supportFound){
            ROS_ERROR("[dialogue_node] Missing support for place action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " placed the " + objectName + " on the " + supportName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " can not place the " + objectName + " on the " + supportName;
        }else if(actionState == "NOT_PERFORMED"){
            sentence = actorName + " did not place the " + objectName + " on the " + supportName;
        }else if(actionState == "WILL"){
            sentence = actorName + " will place the " + objectName + " on the " + supportName;
        }else if(actionState == "SHOULD"){
            sentence = actorName + " should place the " + objectName + " on the " + supportName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        std::string objectTopic, containerTopic, actorTopic;
        std::string objectName, containerName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        bool containerFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "container"){
                containerTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(containerTopic, containerName);
                containerFound = true;
            }
        }
        if(!containerFound){
            ROS_ERROR("[dialogue_node] Missing container for drop action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " put the " + objectName + " in the " + containerName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " can not put the " + objectName + " in the " + containerName;
        }else if(actionState == "NOT_PERFORMED"){
            sentence = actorName + " did not put the " + objectName + " in the " + containerName;
        }else if(actionState == "WILL"){
            sentence = actorName + " will put the " + objectName + " in the " + containerName;
        }else if(actionState == "SHOULD"){
            sentence = actorName + " will put the " + objectName + " in the " + containerName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "scan"){
        std::string objectTopic, actorTopic;
        std::string objectName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " scanned the " + objectName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " failed to scan the " + objectName;
        }else if(actionState == "NOT_PERFORMED"){
            sentence = actorName + " did not scan " + objectName;
        }else if(actionState == "WILL"){
            sentence = actorName + " will scan " + objectName;
        }else if(actionState == "SHOULD"){
            sentence = actorName + " will scan " + objectName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }

    //We verbalize the sentence
    saySentence(sentence, receiver);

    if(actionState == "DONE"){
        action.succeed = true;
    }else{
        action.succeed = false;
    }
    supervisor_msgs::Info msg;
    msg.toRobot = false;
    msg.agent = receiver;
    msg.type = "ACTION";
    msg.action = action;
    info_pub_.publish(msg);
}


/**
 * \brief Give the state of a plan
 * @param plan the concerned plan
 * @param planState the state of the plan
 * @param receiver the person to talk to
 * */
void giveInfoPlan(supervisor_msgs::SharedPlan plan, std::string planState, std::string receiver){

    std::string sentence;
    if(planState == "DONE"){
        sentence = "The plan is over";
    }else if(planState == "FAILED"){
        sentence = "I aborted the previous plan";
    }else{
       ROS_ERROR("[dialogue_node] Plan state not supported");
    }

    //We verbalize the sentence
    saySentence(sentence, receiver);
}

/**
 * \brief Give the state of a goal
 * @param goal the concerned goal
 * @param receiver the person to talk to
 * */
void giveInfoGoal(std::string goal, std::string receiver){

    std::string sentence = "I am executing the goal " + goal;

    //We verbalize the sentence
    saySentence(sentence, receiver);

    supervisor_msgs::Info msg;
    msg.toRobot = false;
    msg.agent = receiver;
    msg.type = "GOAL";
    msg.goal = goal;
    info_pub_.publish(msg);
}


/**
 * \brief Share a plan
 * @param plan the concerned plan
 * @param receiver the person to talk to (not used for now)
 * */
void sharePlan(supervisor_msgs::SharedPlan plan, std::string receiver){

     /** @todo: add plan verbalization */

    std::string sentence;
    sentence = "The new plan is displayed on the screen";

    //We verbalize the sentence
    saySentence(sentence, receiver);
}


/**
 * \brief Ask to perform an action
 * @param action the action to perform
 * @param receiver the person to talk to (not used for now)
 * */
void askCanAction(supervisor_msgs::Action action, std::string receiver){

    /** @todo: find a better way to do this */

    std::string sentence;
    if(action.name == "pick"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        sentence = "Can you pick the " + objectName +  " please?";
    }else if(action.name == "place" || action.name == "pickandplace"){
        std::string objectTopic, supportTopic, objectName, supportName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for place action");
            return;
        }
        bool supportFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "support"){
                supportTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(supportTopic, supportName);
                supportFound = true;
            }
        }
        if(!supportFound){
            ROS_ERROR("[dialogue_node] Missing support for place action");
            return;
        }
        sentence = "Can you place the " + objectName + " on the " + supportName + " please?";
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        std::string objectTopic, containerTopic, objectName, containerName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        bool containerFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "container"){
                containerTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(containerTopic, containerName);
                containerFound = true;
            }
        }
        if(!containerFound){
            ROS_ERROR("[dialogue_node] Missing container for drop action");
            return;
        }
        sentence = "Can you put the " + objectName + " in the " + containerName + " please?";
    }else if(action.name == "scan"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        sentence = "Can you scan the " + objectName + " please?";
    }

    //We verbalize the sentence
    saySentence(sentence, receiver);
}


/**
 * \brief Ask if the receiver wants to perform an action
 * @param action the action to perform
 * @param receiver the person to talk to (not used for now)
 * */
void askWantAction(supervisor_msgs::Action action, std::string receiver){

    /** @todo find a better way to do this */
    std::string sentence;
    if(action.name == "pick"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        sentence = "Do you want to pick the " + objectName +  "?";
    }else if(action.name == "place" || action.name == "pickandplace"){
        std::string objectTopic, supportTopic, objectName, supportName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for place action");
            return;
        }
        bool supportFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "support"){
                supportTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(supportTopic, supportName);
                supportFound = true;
            }
        }
        if(!supportFound){
            ROS_ERROR("[dialogue_node] Missing support for place action");
            return;
        }
        sentence = "Do you want to place the " + objectName + " on the " + supportName + "?";
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        std::string objectTopic, containerTopic, objectName, containerName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        bool containerFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "container"){
                containerTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(containerTopic, containerName);
                containerFound = true;
            }
        }
        if(!containerFound){
            ROS_ERROR("[dialogue_node] Missing container for drop action");
            return;
        }
        sentence = "Do you want to put the " + objectName + " in the " + containerName + "?";
    }else if(action.name == "scan"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        sentence = "Do you want to scan the " + objectName + "?";
    }

    //We verbalize the sentence
    saySentence(sentence, receiver);
}


/**
 * \brief Ask an information (fact)
 * @param fact the information to ask
 * @param receiver the person to talk to (not used for now)
 * */
void askFact(toaster_msgs::Fact fact, std::string receiver){

    //We transform the fact into a sentence
    std::string sentence;
    std::string subjectTopic, targetTopic, propertyTopic;
    std::string subjectName, targetName, propertyName;
    if(fact.subjectId == robotName_){
        subjectName = "I";
    }else if(fact.subjectId == receiver){
        subjectName = "you";
    }else{
        subjectTopic = "dialogue_node/entitiesTranslation/" + fact.subjectId;
        node_->getParam(subjectTopic, subjectName);
    }
    if(fact.targetId == robotName_){
        targetName = "me";
    }else if(fact.targetId == receiver ){
        targetName = "you";
    }else{
        targetTopic = "dialogue_node/entitiesTranslation/" + fact.targetId;
        node_->getParam(targetTopic, targetName);
    }
    propertyTopic = "dialogue_node/properties/translationAsk/" + fact.property;
    node_->getParam(propertyTopic, propertyName);
    sentence = "Is the " + subjectName + " " + propertyName + " " + targetName + "?";

    //We verbalize the sentence
    saySentence(sentence, receiver);
}


/**
 * \brief Service call to say a sentence
 * @param req the request of the service
 * @param res the result of the service
 * @return true
 * */
bool say(supervisor_msgs::String::Request  &req, supervisor_msgs::String::Response &res){

    saySentence(req.data, "NONE");

    return true;
}


/**
 * \brief Service call to give an information or when we get an information
 * @param req the request of the service
 * @param res the result of the service
 * @return true
 * */
bool giveInfo(supervisor_msgs::GiveInfo::Request  &req, supervisor_msgs::GiveInfo::Response &res){

    if(req.toRobot){
        if(req.type == "FACT"){
            //We inform the mental state
            supervisor_msgs::Info msg;
            msg.toRobot = true;
            msg.agent = req.partner;
            msg.type = "FACT";
            msg.fact = req.fact;
            msg.isTrue = req.isTrue;
            info_pub_.publish(msg);
        }else{
            ROS_ERROR("[dialogue_node] Unknown type of given info");
            return false;
        }
    }else{
        if(req.type == "FACT"){
            std::pair<std::string, std::pair<toaster_msgs::Fact, bool> >  toPush;
            toPush.second.first = req.fact;
            toPush.second.second = req.isTrue;
            toPush.first = req.partner;
            toSayFacts.push_back(toPush);
        }else if(req.type == "ACTION"){
            std::pair<std::string, std::pair<supervisor_msgs::Action, std::string> > toPush;
            toPush.second.first = req.action;
            toPush.second.second = req.actionState;
            toPush.first = req.partner;
            toSayActions.push_back(toPush);
        }else if(req.type == "PLAN"){
            if(req.planState == "SHARE"){
                std::pair<std::string, supervisor_msgs::SharedPlan> toPush;
                toPush.first = req.partner;
                toPush.second = req.plan;
                toSharePlans.push_back(toPush);
            }else{
                std::pair<std::string, std::pair<supervisor_msgs::SharedPlan, std::string> > toPush;
                toPush.second.first = req.plan;
                toPush.second.second = req.planState;
                toPush.first = req.partner;
                toSayPlans.push_back(toPush);
            }
        }else if(req.type == "GOAL"){
                giveInfoGoal(req.goal, req.partner);
        }
    }

    return true;
}


/**
 * \brief Service to ask something
 * @param req the request of the service
 * @param res the result of the service
 * @return true
 * */
bool ask(supervisor_msgs::Ask::Request  &req, supervisor_msgs::Ask::Response &res){


    if(req.type == "ACTION"){
        if(req.subType == "CAN"){
            askCanAction(req.action, req.receiver);
        }else if(req.subType == "WANT"){
            askWantAction(req.action, req.receiver);
        }
        supervisor_msgs::Info msg;
        msg.toRobot = false;
        msg.agent = req.receiver;
        msg.type = "ASK_ACTION";
        msg.action = req.action;
        info_pub_.publish(msg);
    }else if(req.type == "FACT"){
        askFact(req.fact, req.receiver);
    }else{
        ROS_ERROR("[dialogue_node] Unknown ask type");
        return false;
    }

    if(req.waitForAnswer){
        std_msgs::Bool answer = *(ros::topic::waitForMessage<std_msgs::Bool>("graphical_interface/boolAnswer",ros::Duration(timeToWait_)));
        res.boolAnswer = answer.data;
        ROS_INFO("[DIALOGUE] ANSWER: %s", (res.boolAnswer)?"yes":"no");
    }

    return true;
}


#ifdef ACAPELA
/**
 * \brief Initialize acapela
 * */
void initAcapela(){
    //Init
    ROS_INFO("[dialogue_node] Waiting for acapela init");
    node_->getParam("dialogue_node/waitActionServer", waitActionServer);
    acInit = new actionlib::SimpleActionClient<acapela::InitAction>("acapela/Init", true);
    acInit->waitForServer();
    acapela::InitGoal goal;
    goal.server = "maxc2";
    acInit->sendGoal(goal);
    bool finishedBeforeTimeout = acInit->waitForResult(ros::Duration(waitActionServer));
    if (!finishedBeforeTimeout){
       ROS_INFO("Acapela init did not finish before the time out.");
    }

    ROS_INFO("[dialogue_node] Waiting for acapela set voice");
    //Set voice
    std::string voice;
    node_->getParam("dialogue_node/acapelaVoice", voice);
    ros::ServiceClient client = node_->serviceClient<acapela::SetVoice>("acapela/SetVoice");
    acapela::SetVoice srv;
    srv.request.voice = voice;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service acapela/SetVoice");
    }
    
    ROS_INFO("[dialogue_node] Waiting for acapela say");
	acSay = new actionlib::SimpleActionClient<acapela::SayAction>("acapela/Say", true);
	acSay->waitForServer();

}
#endif //ACAPELA

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dialogue_node");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate loop_rate(30);
  node_->getParam("dialogue_node/shouldSpeak", shoudlSpeak_);
  node_->getParam("supervisor/robot/name", robotName_);
  node_->getParam("dialogue_node/timeWaitAnswer", timeToWait_);

  //Services declarations
  ros::ServiceServer service_say = node_->advertiseService("dialogue_node/say", say); //say a sentence
  ros::ServiceServer service_give = node_->advertiseService("dialogue_node/give_info", giveInfo); //give an information
  ros::ServiceServer service_ask = node_->advertiseService("dialogue_node/ask", ask); //give an information


  info_pub_ = node_->advertise<supervisor_msgs::Info>("/dialogue_node/infoGiven", 1);
  speaking_pub_ = node_->advertise<std_msgs::Bool>("/dialogue_node/isSpeaking", 1);

  #ifdef ACAPELA
  initAcapela();
  #endif //ACAPELA

  ROS_INFO("[dialogue_node] dialogue_node ready");

  while (node.ok()) {
      ros::spinOnce();
      for(std::vector<std::pair<std::string, std::pair<toaster_msgs::Fact, bool> > >::iterator it = toSayFacts.begin(); it != toSayFacts.end(); it++){
        giveInfoFact(it->second.first, it->second.second, it->first);
      }
      toSayFacts.clear();
      for(std::vector<std::pair<std::string, std::pair<supervisor_msgs::Action, std::string> > >::iterator it = toSayActions.begin(); it != toSayActions.end(); it++){
        giveInfoAction(it->second.first, it->second.second, it->first);
      }
      toSayActions.clear();
      for(std::vector<std::pair<std::string, std::pair<supervisor_msgs::SharedPlan, std::string> > >::iterator it = toSayPlans.begin(); it != toSayPlans.end(); it++){
        giveInfoPlan(it->second.first, it->second.second, it->first);
      }
      toSayPlans.clear();
      for(std::vector<std::pair<std::string, supervisor_msgs::SharedPlan> >::iterator it = toSharePlans.begin(); it != toSharePlans.end(); it++){
        sharePlan(it->second, it->first);
      }
      toSharePlans.clear();
      loop_rate.sleep();
  }

  return 0;
}

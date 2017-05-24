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
#include <fstream>

#include "std_msgs/Bool.h"

#include "supervisor_msgs/GiveInfo.h"
#include "supervisor_msgs/Ask.h"
#include "supervisor_msgs/SharedPlan.h"
#include "supervisor_msgs/String.h"
#include "supervisor_msgs/Info.h"
#include "supervisor_msgs/GoalsList.h"


#ifdef ACAPELA
#include "acapela/InitAction.h"
#include "acapela/SetVoice.h"
#include "acapela/SayAction.h"
#endif //ACAPELA

ros::NodeHandle* node_;
bool shoudlSpeak_;
std::string robotName_;
double timeToWait_;
bool french_;
std::vector<std::pair<std::string, std::pair<toaster_msgs::Fact, bool> > > toSayFacts;
std::vector<std::pair<std::string, std::pair<supervisor_msgs::Action, std::string> > > toSayActions;
std::vector<std::pair<std::string, std::pair<supervisor_msgs::SharedPlan, std::string> > > toSayPlans;
std::vector<std::pair<std::string, supervisor_msgs::SharedPlan> > toSharePlans;

ros::Publisher info_pub_;
ros::Publisher speaking_pub_;

std::string nbParticipant_;
std::string condition_;
int nbQuestion_ = 0;
int nbInfo_ = 0;
bool started = false;

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

    nbInfo_++;

    //We transform the fact into a sentence
    std::string sentence;
    std::string isReverseTopic, subjectTopic, targetTopic, propertyTopic;
    std::string subjectName, targetName, propertyName;
    bool isReverse;
    isReverseTopic = "dialogue_node/properties/isReverse/" + fact.property;
    node_->getParam(isReverseTopic, isReverse);
    if(fact.subjectId == robotName_){
        if(isReverse){
            if(!french_){
                subjectName = "I";
            }else{
                subjectName = "Je";
            }
        }else{
            if(!french_){
                subjectName = "me";
            }else{
                subjectName = "moi";
            }
        }
    }else if(fact.subjectId == receiver){
        if(isReverse){
            if(!french_){
                subjectName = "You";
            }else{
                subjectName = "Vous";
            }
        }else{
            if(!french_){
                subjectName = "you";
            }else{
                subjectName = "votre";
            }
        }
    }else{
        if(!french_){
            subjectTopic = "dialogue_node/entitiesTranslation/" + fact.subjectId;
        }else{
            subjectTopic = "dialogue_node/entitiesTranslationFrench/" + fact.subjectId;
        }
        node_->getParam(subjectTopic, subjectName);
    }
    if(fact.targetId == robotName_){
        if(isReverse){
            if(!french_){
                targetName = "I";
            }else{
                targetName = "Je";
            }
        }else{
            if(!french_){
                targetName = "me";
            }else{
                targetName = "moi";
            }
        }
    }else if(fact.targetId == receiver){
        if(isReverse){
            if(!french_){
                targetName = "You";
            }else{
                targetName = "Vous";
            }
        }else{
            if(!french_){
                targetName = "you";
            }else{
                targetName = "votre";
            }
        }
    }else{
        if(!french_){
            targetTopic = "dialogue_node/entitiesTranslation/" + fact.targetId;
        }else{
            targetTopic = "dialogue_node/entitiesTranslationFrench/" + fact.targetId;
        }
        node_->getParam(targetTopic, targetName);
    }
    if(isTrue){
        if(!french_){
            propertyTopic = "dialogue_node/properties/translationTrue/" + fact.property;
        }else{
            propertyTopic = "dialogue_node/properties/translationTrueFrench/" + fact.property;
        }
    }else{
        if(!french_){
            propertyTopic = "dialogue_node/properties/translationFalse/" + fact.property;
        }else{
            propertyTopic = "dialogue_node/properties/translationFalseFrench/" + fact.property;
        }
    }
    node_->getParam(propertyTopic, propertyName);
    if(isReverse){
        if(!french_){
            sentence = targetName + " " + propertyName + " the " + subjectName;
        }else{
            sentence = targetName + " " + propertyName + " " + subjectName;
        }
    }else{
        if(!french_){
            sentence = "The " + subjectName + " " + propertyName + " the " + targetName;
        }else{
            sentence = subjectName + " " + propertyName + " " + targetName;
        }
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


    nbInfo_++;

    /** @todo: find a better way to do this */
    std::string sentence;
    if(action.name == "pick"){
        std::string objectTopic, actorTopic;
        std::string objectName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                if(!french_){
                    actorName = "I";
                }else{
                    actorName = "Je";
                }
            }else if(action.actors[0] == receiver){
                if(!french_){
                    actorName = "you";
                }else{
                    actorName = "Vous";
                }
            }else{
                if(!french_){
                    actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                }else{
                    actorTopic = "dialogue_node/entitiesTranslationFrench/" + action.actors[0];
                }
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                /*if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);*/
                if(action.parameter_values[i] == "RED_TAPE1" || action.parameter_values[i] == "RED_TAPE2"){
                    if(!french_){
                        objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                    }else{
                        objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                    }
                    node_->getParam(objectTopic, objectName);
                }else{
                    if(!french_){
                        objectName = "next cube";
                    }else{
                        objectTopic = "le prochain cube";
                    }
                }
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        if(actionState == "DONE"){
            if(!french_){
                sentence = actorName + " picked the " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "J'ai pris le " + objectName;
                }else{
                    sentence = "Vous avez pris " + objectName;
                }
            }
        }else if(actionState == "FAILED"){
            if(!french_){
                sentence = actorName + " can not pick the " + objectName;
            }else{
                sentence = actorName + " ne peux pas prendre " + objectName;
            }
        }else if(actionState == "NOT_PERFORMED"){
            if(!french_){
                sentence = actorName + " did not pick the " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je n'ai pas pris " + objectName;
                }else{
                    sentence = "Vous n'avez pas pris " + objectName;
                }
            }
        }else if(actionState == "WILL"){
            if(!french_){
                sentence = actorName + " will pick the " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je vez prendre " + objectName;
                }else{
                    sentence = "Vous allez prendre " + objectName;
                }
            }
        }else if(actionState == "SHOULD"){
            if(!french_){
                sentence = actorName + " should pick the " + objectName;
            }else{
                sentence = actorName + " devriez prendre " + objectName;
            }
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "place" || action.name == "pickandplace"){
        std::string objectTopic, supportTopic, actorTopic;
        std::string objectName, supportName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                if(!french_){
                    actorName = "I";
                }else{
                    actorName = "Je";
                }
            }else if(action.actors[0] == receiver){
                if(!french_){
                    actorName = "you";
                }else{
                    actorName = "Tu";
                }
            }else{
                if(!french_){
                    actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                }else{
                    actorTopic = "dialogue_node/entitiesTranslationFrench/" + action.actors[0];
                }
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                /*if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);*/
                if(action.parameter_values[i] == "RED_TAPE1" || action.parameter_values[i] == "RED_TAPE2"){
                    if(!french_){
                        objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                    }else{
                        objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                    }
                    node_->getParam(objectTopic, objectName);
                }else{
                    if(!french_){
                        objectName = "next cube";
                    }else{
                        objectTopic = "le prochain cube";
                    }
                }
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
                if(!french_){
                    supportTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    supportTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(supportTopic, supportName);
                supportFound = true;
            }
        }
        if(!supportFound){
            ROS_ERROR("[dialogue_node] Missing support for place action");
            return;
        }
        if(actionState == "DONE"){
            if(!french_){
                sentence = actorName + " placed the " + objectName + " on the " + supportName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "J'ai posé le " + objectName + " sur le " + supportName;
                }else{
                    sentence = "Vous avez posé " + objectName + " sur " + supportName;
                }
            }
        }else if(actionState == "FAILED"){
            if(!french_){
                sentence = actorName + " can not place the " + objectName + " on the " + supportName;
            }else{
                sentence = actorName + " ne peux pas poser " + objectName + " sur " + supportName;
            }
        }else if(actionState == "NOT_PERFORMED"){
            if(!french_){
                sentence = actorName + " did not place the " + objectName + " on the " + supportName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je n'ai pas posé " + objectName + " sur " + supportName;
                }else{
                    sentence = "Vous n'avez pas posé " + objectName + " sur " + supportName;
                }
            }
        }else if(actionState == "WILL"){
            if(!french_){
                sentence = actorName + " will place the " + objectName + " on the " + supportName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je vez poser " + objectName + " sur " + supportName;
                }else{
                    sentence = "Vous allez poser " + objectName + " sur " + supportName;
                }
            }
        }else if(actionState == "SHOULD"){
            if(!french_){
                sentence = actorName + " should place the " + objectName + " on the " + supportName;
            }else{
                sentence = actorName + " devriez poser " + objectName + " sur " + supportName;
            }
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        std::string objectTopic, containerTopic, actorTopic;
        std::string objectName, containerName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                if(!french_){
                    actorName = "I";
                }else{
                    actorName = "Je";
                }
            }else if(action.actors[0] == receiver){
                if(!french_){
                    actorName = "you";
                }else{
                    actorName = "Vous";
                }
            }else{
                if(!french_){
                    actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                }else{
                    actorTopic = "dialogue_node/entitiesTranslationFrench/" + action.actors[0];
                }
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
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
                if(!french_){
                    containerTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    containerTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(containerTopic, containerName);
                containerFound = true;
            }
        }
        if(!containerFound){
            ROS_ERROR("[dialogue_node] Missing container for drop action");
            return;
        }
        if(actionState == "DONE"){
            if(!french_){
                sentence = actorName + " put the " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "J'ai rangé " + objectName;
                }else{
                    sentence = "Vous avez rangé " + objectName;
                }
            }
        }else if(actionState == "FAILED"){
            if(!french_){
                sentence = actorName + " can not put the " + objectName;
            }else{
                sentence = actorName + " ne peux pas ranger " + objectName;
            }
        }else if(actionState == "NOT_PERFORMED"){
            if(!french_){
                sentence = actorName + " did not put the " + objectName ;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je n'ai pas rangé le " + objectName;
                }else{
                    sentence = "Vous n'avez pas rangé " + objectName;
                }
            }
        }else if(actionState == "WILL"){
            if(!french_){
                sentence = actorName + " will put the " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je vez ranger " + objectName;
                }else{
                    sentence = "Vous allez ranger " + objectName;
                }
            }
        }else if(actionState == "SHOULD"){
            if(!french_){
                sentence = actorName + " will put the " + objectName;
            }else{
                sentence = actorName + " devriez ranger " + objectName;
            }
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "scan"){
        std::string objectTopic, actorTopic;
        std::string objectName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName_){
                if(!french_){
                    actorName = "I";
                }else{
                    actorName = "Je";
                }
            }else if(action.actors[0] == receiver){
                if(!french_){
                    actorName = "you";
                }else{
                    actorName = "Vous";
                }
            }else{
                if(!french_){
                    actorTopic = "dialogue_node/entitiesTranslation/" + action.actors[0];
                }else{
                    actorTopic = "dialogue_node/entitiesTranslationFrench/" + action.actors[0];
                }
                node_->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        if(actionState == "DONE"){
            if(!french_){
                sentence = actorName + " scanned the " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "J'ai scanné " + objectName;
                }else{
                    sentence = "Vous avez scanné " + objectName;
                }
            }
        }else if(actionState == "FAILED"){
            if(!french_){
                sentence = actorName + " failed to scan the " + objectName;
            }else{
                sentence = actorName + " ne peux pas scanner " + objectName;
            }
        }else if(actionState == "NOT_PERFORMED"){
            if(!french_){
                sentence = actorName + " did not scan " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je n'ai pas scanné " + objectName;
                }else{
                    sentence = "Vous n'avez pas scanné " + objectName;
                }
            }
        }else if(actionState == "WILL"){
            if(!french_){
                sentence = actorName + " will scan " + objectName;
            }else{
                if(action.actors[0] == robotName_){
                    sentence = "Je vez scanner " + objectName;
                }else{
                    sentence = "Vous allez scanner " + objectName;
                }
            }
        }else if(actionState == "SHOULD"){
            if(!french_){
                sentence = actorName + " will scan " + objectName;
            }else{
                sentence = actorName + " devriez scanner " + objectName;
            }
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
        if(!french_){
            sentence = "The plan is over";
        }else{
            sentence = "Le plan est terminé";
        }
    }else if(planState == "FAILED"){
        if(!french_){
            sentence = "I aborted the previous plan";
        }else{
            sentence = "J'ai abandonné le plan précédent";
        }
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

    std::string sentence;
    if(!french_){
        sentence = "I am executing the goal " + goal;
    }else{
        sentence = "J'execute le goal " + goal;
    }

    //We verbalize the sentence
    //saySentence(sentence, receiver);

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
    if(!french_){
        sentence = "The new plan is displayed on the screen";
    }else{
        sentence = "Le nouveau plan est affiché sur l'ecran";
    }

    //We verbalize the sentence
    saySentence(sentence, receiver);
}


/**
 * \brief Ask to perform an action
 * @param action the action to perform
 * @param receiver the person to talk to (not used for now)
 * */
void askCanAction(supervisor_msgs::Action action, std::string receiver){

    nbQuestion_++;

    /** @todo: find a better way to do this */

    std::string sentence;
    if(action.name == "pick"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                /*if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);*/
                if(action.parameter_values[i] == "RED_TAPE1" || action.parameter_values[i] == "RED_TAPE2"){
                    if(!french_){
                        objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                    }else{
                        objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                    }
                    node_->getParam(objectTopic, objectName);
                }else{
                    if(!french_){
                        objectName = "next cube";
                    }else{
                        objectTopic = "le prochain cube";
                    }
                }
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        if(!french_){
            sentence = "Can you pick the " + objectName +  " please?";
        }else{
            sentence = "Pouvez vous prendre " + objectName +  " s'il vous plait?";
        }
    }else if(action.name == "place" || action.name == "pickandplace"){
        std::string objectTopic, supportTopic, objectName, supportName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                /*if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);*/
                if(action.parameter_values[i] == "RED_TAPE1" || action.parameter_values[i] == "RED_TAPE2"){
                    if(!french_){
                        objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                    }else{
                        objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                    }
                    node_->getParam(objectTopic, objectName);
                }else{
                    if(!french_){
                        objectName = "next cube";
                    }else{
                        objectTopic = "le prochain cube";
                    }
                }
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
                if(!french_){
                    supportTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    supportTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(supportTopic, supportName);
                supportFound = true;
            }
        }
        if(!supportFound){
            ROS_ERROR("[dialogue_node] Missing support for place action");
            return;
        }
        if(!french_){
            sentence = "Can you place the " + objectName + " on the " + supportName + " please?";
        }else{
            sentence = "Pouvez vous placer " + objectName + " sur " + supportName + " s'il vous plait?";
        }
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        std::string objectTopic, containerTopic, objectName, containerName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
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
                if(!french_){
                    containerTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    containerTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(containerTopic, containerName);
                containerFound = true;
            }
        }
        if(!containerFound){
            ROS_ERROR("[dialogue_node] Missing container for drop action");
            return;
        }
        if(!french_){
            sentence = "Can you put the " + objectName + " please?";
        }else{
            sentence = "Pouvez vous ranger " + objectName + " s'il vous plait?";
        }
    }else if(action.name == "scan"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        if(!french_){
            sentence = "Can you scan the " + objectName + " please?";
        }else{
            sentence = "Pouvez vous scanner " + objectName +  " s'il vous plait?";
        }
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

    nbQuestion_++;

    /** @todo find a better way to do this */
    std::string sentence;
    if(action.name == "pick"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        if(!french_){
            sentence = "Do you want to pick the " + objectName +  "?";
        }else{
            sentence = "Voulez vous prendre " + objectName +  "?";
        }
    }else if(action.name == "place" || action.name == "pickandplace"){
        std::string objectTopic, supportTopic, objectName, supportName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
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
                if(!french_){
                    supportTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    supportTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(supportTopic, supportName);
                supportFound = true;
            }
        }
        if(!supportFound){
            ROS_ERROR("[dialogue_node] Missing support for place action");
            return;
        }
        if(!french_){
            sentence = "Do you want to place the " + objectName + " on the " + supportName + "?";
        }else{
            sentence = "Voulez vous poser " + objectName + " sur " + supportName + "?";
        }
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        std::string objectTopic, containerTopic, objectName, containerName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
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
                if(!french_){
                    containerTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    containerTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(containerTopic, containerName);
                containerFound = true;
            }
        }
        if(!containerFound){
            ROS_ERROR("[dialogue_node] Missing container for drop action");
            return;
        }
        if(!french_){
            sentence = "Do you want to put the " + objectName + "?";
        }else{
            sentence = "Voulez vous ranger " + objectName + "?";
        }
    }else if(action.name == "scan"){
        std::string objectTopic, objectName;
        bool objectFound = false;
        for(int i = 0; i < action.parameter_keys.size(); i++){
            if(action.parameter_keys[i] == "object"){
                if(!french_){
                    objectTopic = "dialogue_node/entitiesTranslation/" + action.parameter_values[i];
                }else{
                    objectTopic = "dialogue_node/entitiesTranslationFrench/" + action.parameter_values[i];
                }
                node_->getParam(objectTopic, objectName);
                objectFound = true;
            }
        }
        if(!objectFound){
            ROS_ERROR("[dialogue_node] Missing object for drop action");
            return;
        }
        if(!french_){
            sentence = "Do you want to scan the " + objectName + "?";
        }else{
            sentence = "Voulez vous scanner " + objectName +  "?";
        }
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
        if(!french_){
            subjectName = "I";
        }else{
            subjectName = "Je";
        }
    }else if(fact.subjectId == receiver){
        if(!french_){
            subjectName = "you";
        }else{
            subjectName = "Vous";
        }
    }else{
        if(!french_){
            subjectTopic = "dialogue_node/entitiesTranslation/" + fact.subjectId;
        }else{
            subjectTopic = "dialogue_node/entitiesTranslationFrench/" + fact.subjectId;
        }
        node_->getParam(subjectTopic, subjectName);
    }
    if(fact.targetId == robotName_){
        if(!french_){
            targetName = "me";
        }else{
            subjectName = "moi";
        }
    }else if(fact.targetId == receiver ){
        if(!french_){
            targetName = "you";
        }else{
            subjectName = "vous";
        }
    }else{
        if(!french_){
            targetTopic = "dialogue_node/entitiesTranslation/" + fact.targetId;
        }else{
            targetTopic = "dialogue_node/entitiesTranslationFrench/" + fact.targetId;
        }
        node_->getParam(targetTopic, targetName);
    }
    propertyTopic = "dialogue_node/properties/translationAsk/" + fact.property;
    node_->getParam(propertyTopic, propertyName);
    if(!french_){
        sentence = "Is the " + subjectName + " " + propertyName + " " + targetName + "?";
    }else{
        sentence = "Est ce que " + subjectName + " " + propertyName + " " + targetName + "?";
    }

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
    goal.server = "bobc2";
    acInit->sendGoal(goal);
    bool finishedBeforeTimeout = acInit->waitForResult(ros::Duration(waitActionServer));
    if (!finishedBeforeTimeout){
       ROS_INFO("Acapela init did not finish before the time out.");
    }

    ROS_INFO("[dialogue_node] Waiting for acapela set voice");
    //Set voice
    std::string voice;
    if(!french_){
        node_->getParam("dialogue_node/acapelaVoice", voice);
    }else{
        node_->getParam("dialogue_node/acapelaVoiceFrench", voice);
    }
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

/**
 * \brief Callback of the goal list
 * @param msg topic msg
 * */
void goalsListCallback(const supervisor_msgs::GoalsList::ConstPtr& msg){

    if(msg->changed){
        if(msg->currentGoal == "SCAN_US" && !started){
            started = true;
        }
        if(msg->currentGoal == "NONE" && started){
            started = false;
            //we log results
            std::ofstream fileSave;
            //we save the execution time
            std::string fileName = "/home/sdevin/catkin_ws/src/supervisor/logs/Verb.txt";
            fileSave.open(fileName.c_str(), std::ios::out|std::ios::ate);
            std::ostringstream strs;
            strs << nbQuestion_;
            std::string nbQ = strs.str();
            std::ostringstream strs2;
            strs2 << nbInfo_;
            std::string nbInfo = strs2.str();
            fileSave << "Participant " << nbParticipant_.c_str() << "\t Condition " << condition_.c_str() << "\t " << nbInfo.c_str() << "\t" << nbQ.c_str() << std::endl;

        }
    }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dialogue_node");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate loop_rate(30);
  node_->getParam("dialogue_node/shouldSpeak", shoudlSpeak_);
  node_->getParam("supervisor/robot/name", robotName_);
  node_->getParam("dialogue_node/timeWaitAnswer", timeToWait_);
  node_->getParam("dialogue_node/french", french_);
  node_->getParam("/supervisor/nbParticipant", nbParticipant_);

  std::string systemMode;
  node_->getParam("/supervisor/systemMode", systemMode);
  if(systemMode == "new"){
      std::string speakMode;
      node_->getParam("/robot_decision/mode", speakMode);
      if(speakMode == "negotiation"){
        condition_ = "Negotiation";
      }else{
        condition_ = "Adaptation";
      }
  }else{
      bool speakMode;
      node_->getParam("/supervisor/speakingMode", speakMode);
      if(speakMode){
        condition_ = "RS-all";
      }else{
        condition_ = "RS-none";
      }
  }

  ros::Subscriber sub_goal = node.subscribe("/goal_manager/goalsList", 1, goalsListCallback);

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

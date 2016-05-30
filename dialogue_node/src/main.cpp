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

#include "supervisor_msgs/Say.h"
#include "supervisor_msgs/GiveInfo.h"
#include "supervisor_msgs/InfoGiven.h"
#include "supervisor_msgs/ChangeState.h"


#ifdef ACAPELA
#include "acapela/InitAction.h"
#include "acapela/SetVoice.h"
#include "acapela/SayAction.h"
#endif //ACAPELA

using namespace std;

ros::NodeHandle* node;
bool shoudlSpeak;
string robotName;
vector<pair<string, pair<toaster_msgs::Fact, bool> > > toSayFacts;
vector<pair<string, pair<supervisor_msgs::Action, string> > > toSayActions;
vector<pair<string, pair<supervisor_msgs::Plan, string> > > toSayPlans;
vector<pair<string, supervisor_msgs::Plan> > toSharePlans;

#ifdef ACAPELA
double waitActionServer;
actionlib::SimpleActionClient<acapela::InitAction>* acInit;
actionlib::SimpleActionClient<acapela::SayAction>* acSay;
#endif //ACAPELA

/*
Verbalize it if acapela is defined
*/
void saySentence(string sentence){

    ROS_INFO("[dialogue_node] ROBOT: %s", sentence.c_str());

    #ifdef ACAPELA
    if(shoudlSpeak){
        acapela::SayGoal goal;
        acSay->sendGoal(goal);
        bool finishedBeforeTimeout = acSay->waitForResult(ros::Duration(waitActionServer));
        if (!finishedBeforeTimeout){
           ROS_INFO("Acapela say did not finish before the time out.");
        }
    }
    #endif //ACAPELA

}

/*
Verbalize a fact
*/
void giveInfoFact(toaster_msgs::Fact fact, bool isTrue, string receiver){

    //We transform the fact into a sentence
    string sentence;
    string isReverseTopic, subjectTopic, targetTopic, propertyTopic;
    string subjectName, targetName, propertyName;
    bool isReverse;
    isReverseTopic = "/properties/isReverse/" + fact.property;
    node->getParam(isReverseTopic, isReverse);
    if(fact.subjectId == robotName){
        if(isReverse){
            subjectName = "I";
        }else{
            subjectName = "me";
        }
    }else if(fact.subjectId == receiver){
        subjectName = "you";
    }else{
        subjectTopic = "/entitiesTranslation/" + fact.subjectId;
        node->getParam(subjectTopic, subjectName);
    }
    if(fact.targetId == robotName){
        if(isReverse){
            targetName = "I";
        }else{
            targetName = "me";
        }
    }else if(fact.targetId == receiver){
        targetName = "you";
    }else{
        targetTopic = "/entitiesTranslation/" + fact.targetId;
        node->getParam(targetTopic, targetName);
    }
    if(isTrue){
        propertyTopic = "/properties/translationTrue/" + fact.property;
    }else{
        propertyTopic = "/properties/translationFalse/" + fact.property;
    }
    node->getParam(propertyTopic, propertyName);
    if(isReverse){
        sentence = subjectName + " " + propertyName + " the " + targetName;
    }else{
        sentence = "The " + subjectName + " " + propertyName + " the " + targetName;
    }

    //We verbalize the sentence
    saySentence(sentence);

    //We inform the mental state we gave the information
    ros::ServiceClient client = node->serviceClient<supervisor_msgs::InfoGiven>("mental_states/info_given");
    supervisor_msgs::InfoGiven srv;

    srv.request.infoType = "fact";
    srv.request.fact = fact;
    srv.request.receiver = receiver;
    srv.request.sender = robotName;
    if (!client.call(srv)){
       ROS_ERROR("[dialogue_node] Failed to call service mental_states/info_given");
    }
}

/*
Give the state of an action
*/
void giveInfoAction(supervisor_msgs::Action action, string actionState, string receiver){

    //TODO: find a better way to do this
    string sentence;
    if(action.name == "pick"){
        string objectTopic, actorTopic;
        string objectName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "/entitiesTranslation/" + action.actors[0];
                node->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        if(action.parameters.size() > 0){
            objectTopic = "/entitiesTranslation/" + action.parameters[0];
            node->getParam(objectTopic, objectName);
        }else{
            ROS_ERROR("[dialogue_node] Missing object for pick action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " picked the " + objectName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " can not pick the " + objectName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "place" || action.name == "pickandplace"){
        string objectTopic, supportTopic, actorTopic;
        string objectName, supportName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "/entitiesTranslation/" + action.actors[0];
                node->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        if(action.parameters.size() > 1){
            objectTopic = "/entitiesTranslation/" + action.parameters[0];
            node->getParam(objectTopic, objectName);
            supportTopic = "/entitiesTranslation/" + action.parameters[1];
            node->getParam(supportTopic, supportName);
        }else{
            ROS_ERROR("[dialogue_node] Missing object for place action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " placed the " + objectName + " on the " + supportName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " can not place the " + objectName + " on the " + supportName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }else if(action.name == "drop" || action.name == "pickanddrop"){
        string objectTopic, containerTopic, actorTopic;
        string objectName, containerName, actorName;
        if(action.actors.size() > 0){
            if(action.actors[0] == robotName){
                actorName = "I";
            }else if(action.actors[0] == receiver){
                actorName = "you";
            }else{
                actorTopic = "/entitiesTranslation/" + action.actors[0];
                node->getParam(actorTopic, actorName);
            }
        }else{
            ROS_ERROR("[dialogue_node] Missing actor for action");
            return;
        }
        if(action.parameters.size() > 1){
            objectTopic = "/entitiesTranslation/" + action.parameters[0];
            node->getParam(objectTopic, objectName);
            containerTopic = "/entitiesTranslation/" + action.parameters[1];
            node->getParam(containerTopic, containerName);
        }else{
            ROS_ERROR("[dialogue_node] Missing object for place action");
            return;
        }
        if(actionState == "DONE"){
            sentence = actorName + " put the " + objectName + " in the " + containerName;
        }else if(actionState == "FAILED"){
            sentence = actorName + " can not put the " + objectName + " in the " + containerName;
        }else{
            ROS_ERROR("[dialogue_node] Action state not supported");
            return;
        }
    }

    //We verbalize the sentence
    saySentence(sentence);

    //We inform the mental state we gave the information
    ros::ServiceClient client = node->serviceClient<supervisor_msgs::InfoGiven>("mental_states/info_given");
    supervisor_msgs::InfoGiven srv;

    srv.request.infoType = "actionState";
    srv.request.actionId = action.id;
    srv.request.receiver = receiver;
    srv.request.sender = robotName;
    if (!client.call(srv)){
       ROS_ERROR("[dialogue_node] Failed to call service mental_states/info_given");
    }
}

/*
Give the state of an action
*/
void giveInfoPlan(supervisor_msgs::Plan plan, string planState, string receiver){

    string sentence;
    if(planState == "DONE"){
        sentence = "The plan is over";
    }else if(planState == "FAILED"){
        sentence = "I aborted the previous plan";
    }else{
       ROS_ERROR("[dialogue_node] Plan state not supported");
    }

    //We verbalize the sentence
    saySentence(sentence);

    //We inform the mental state we gave the information
    ros::ServiceClient client = node->serviceClient<supervisor_msgs::InfoGiven>("mental_states/info_given");
    supervisor_msgs::InfoGiven srv;

    srv.request.infoType = "planState";
    srv.request.planId = plan.id;
    srv.request.receiver = receiver;
    srv.request.sender = robotName;
    if (!client.call(srv)){
       ROS_ERROR("[dialogue_node] Failed to call service mental_states/info_given");
    }
}

/*
Share a plan
*/
void sharePlan(supervisor_msgs::Plan plan, string receiver){

    string sentence;
    //TODO: add plan verbalization
    sentence = "The new plan is displayed on the screen";

    //We verbalize the sentence
    saySentence(sentence);

    //We inform the mental state we gave the information
    ros::ServiceClient client = node->serviceClient<supervisor_msgs::ChangeState>("mental_states/change_state");
    supervisor_msgs::ChangeState srv;

    srv.request.type = "plan";
    srv.request.state = "SHARE";
    if (!client.call(srv)){
       ROS_ERROR("[dialogue_node] Failed to call service mental_states/change_state");
    }
}

/*
Service call to say a sentence
*/
bool say(supervisor_msgs::Say::Request  &req, supervisor_msgs::Say::Response &res){

    saySentence(req.sentence);

}

/*
Service call to give an information
*/
bool giveInfo(supervisor_msgs::GiveInfo::Request  &req, supervisor_msgs::GiveInfo::Response &res){

    if(req.type == "FACT"){
        pair<string, pair<toaster_msgs::Fact, bool> >  toPush;
        toPush.second.first = req.fact;
        toPush.second.second = req.isTrue;
        toPush.first = req.receiver;
        toSayFacts.push_back(toPush);
    }else if(req.type == "ACTION"){
        pair<string, pair<supervisor_msgs::Action, string> > toPush;
        toPush.second.first = req.action;
        toPush.second.second = req.actionState;
        toPush.first = req.receiver;
        toSayActions.push_back(toPush);
    }else if(req.type == "PLAN"){
        if(req.planState == "SHARE"){
            pair<string, supervisor_msgs::Plan> toPush;
            toPush.first = req.receiver;
            toPush.second = req.plan;
            toSharePlans.push_back(toPush);
        }else{
            pair<string, pair<supervisor_msgs::Plan, string> > toPush;
            toPush.second.first = req.plan;
            toPush.second.second = req.planState;
            toPush.first = req.receiver;
            toSayPlans.push_back(toPush);
        }
    }

    return true;
}

#ifdef ACAPELA
void initAcapela(){
    //Init
    node->getParam("/waitActionServer", waitActionServer);
    acInit = new actionlib::SimpleActionClient<acapela::InitAction>("acapela/Init", true);
    acInit->waitForServer();
    acSay = new actionlib::SimpleActionClient<acapela::SayAction>("acapela/Say", true);
    acSay->waitForServer();
    acapela::InitGoal goal;
    acInit->sendGoal(goal);
    acInit->sendGoal(goal);
    bool finishedBeforeTimeout = acInit->waitForResult(ros::Duration(waitActionServer));
    if (!finishedBeforeTimeout){
       ROS_INFO("Acapela init did not finish before the time out.");
    }

    //Set voice
    string voice;
    node->getParam("/acapelaVoice", voice);
    ros::ServiceClient client = node->serviceClient<acapela::SetVoice>("acapela/SetVoice");
    acapela::SetVoice srv;
    srv.request.voice = voice;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service acapela/SetVoice");
    }

}
#endif //ACAPELA

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dialogue_node");
  ros::NodeHandle _node;
  node = &_node;
  ros::Rate loop_rate(30);
  node->getParam("/shouldSpeak", shoudlSpeak);
  node->getParam("/robot/name", robotName);

  //Services declarations
  ros::ServiceServer service_say = node->advertiseService("dialogue_node/say", say); //say a sentence
  ros::ServiceServer service_give = node->advertiseService("dialogue_node/give_info", giveInfo); //give an information

  #ifdef ACAPELA
  initAcapela();
  #endif //ACAPELA

  ROS_INFO("[dialogue_node] dialogue_node ready");

  while (_node.ok()) {
      for(vector<pair<string, pair<toaster_msgs::Fact, bool> > >::iterator it = toSayFacts.begin(); it != toSayFacts.end(); it++){
        giveInfoFact(it->second.first, it->second.second, it->first);
      }
      toSayFacts.clear();
      for(vector<pair<string, pair<supervisor_msgs::Action, string> > >::iterator it = toSayActions.begin(); it != toSayActions.end(); it++){
        giveInfoAction(it->second.first, it->second.second, it->first);
      }
      toSayActions.clear();
      for(vector<pair<string, pair<supervisor_msgs::Plan, string> > >::iterator it = toSayPlans.begin(); it != toSayPlans.end(); it++){
        giveInfoPlan(it->second.first, it->second.second, it->first);
      }
      toSayPlans.clear();
      for(vector<pair<string, supervisor_msgs::Plan> >::iterator it = toSharePlans.begin(); it != toSharePlans.end(); it++){
        sharePlan(it->second, it->first);
      }
      toSharePlans.clear();
      ros::spinOnce();
      loop_rate.sleep();
  }
  ros::spin();

  return 0;
}

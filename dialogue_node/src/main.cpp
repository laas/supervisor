/**
author Sandra Devin

Simple dialogue node

**/

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "supervisor_msgs/Say.h"

using namespace std;

ros::NodeHandle* node;
vector<string> toSay;
bool shoudlSpeak;

/*
Verbalize it if acapela is defined
*/
void saySentence(string sentence){

    ROS_INFO("[dialogue_node] ROBOT: %s", sentence.c_str());

    #ifdef ACAPELA
    if(shoudlSpeak){
        //TODO: verbalize it if acapela is defined

    }
    #endif //ACAPELA

}

/*
Service call to say a sentence
*/
bool say(supervisor_msgs::Say::Request  &req, supervisor_msgs::Say::Response &res){

    toSay.push_back(req.sentence);

    return true;

}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dialogue_node");
  ros::NodeHandle _node;
  node = &_node;
  ros::Rate loop_rate(30);
  node->getParam("/shouldSpeak", shoudlSpeak);

  //Services declarations
  ros::ServiceServer service_say = node->advertiseService("dialogue_node/say", say); //say a sentence

  ROS_INFO("[dialogue_node] dialogue_node ready");

  while(_node.ok()){
    for(vector<string>::iterator it = toSay.begin(); it != toSay.end(); it++){
        saySentence(*it);
    }
    toSay.clear();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

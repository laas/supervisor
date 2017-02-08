#ifndef HUMANMONITOR_H
#define HUMANMONITOR_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/HumanAction.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/SetInfoDB.h"
#include "toaster_msgs/ObjectListStamped.h"


class HumanMonitor{
public:
    HumanMonitor(ros::NodeHandle* node);
    ~HumanMonitor() {};
    void humanPick(std::string agent, std::string object);
    void humanPlace(std::string agent, std::string object, std::string support);
    void humanDrop(std::string agent, std::string object, std::string container);
    void humanPlaceStick(std::string agent, std::string object, std::string support1, std::string support2);
    std::pair<bool, std::string> hasInHand(std::string agent);
    bool isManipulableObject(std::string object);
    bool isSupportObject(std::string support);
    bool isContainerObject(std::string container);
protected:

private:
    ros::NodeHandle* node_; /**< node handle pointer*/
    bool simu_; /**<  flag to indicate simu*/
    std::string robotName_; /**<  name of the robot*/
    std::vector<std::pair<std::string, std::string> > attachments_; /**< objects the agents have in hand*/
    std::vector<std::string> manipulableObjects_; /**< list of manipulable objects*/
    std::vector<std::string> supportObjects_; /**< list of support objects*/
    std::vector<std::string> containerObjects_; /**< list of container objects*/
    std::string humanHand_; /**< name of the human hand*/
    ros::Publisher previous_pub_; /**< publisher of previous actions*/
    ros::Publisher current_pub_; /**< publisher of humans current actions*/
    ros::ServiceClient client_put_hand_; /**< toaster client to put objects in hand*/
    ros::ServiceClient client_remove_hand_; /**< toaster client to remove objects from hand*/
    ros::ServiceClient client_set_pose_; /**< toaster client to set entities poses*/
    ros::ServiceClient client_set_info_; /**< toaster client to set info in the database*/
};

#endif // HUMANMONITOR_H

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
#include "toaster_msgs/ObjectListStamped.h"


class HumanMonitor{
public:
    HumanMonitor(ros::NodeHandle* node);
    ~HumanMonitor() {};
    void humanPick(std::string agent, std::string object);
    void humanPlace(std::string agent, std::string object, std::string support);
    void humanDrop(std::string agent, std::string object, std::string container);
    std::pair<bool, std::string> hasInHand(std::string agent);
    bool isManipulableObject(std::string object);
    bool isSupportObject(std::string support);
    bool isContainerObject(std::string container);
protected:

private:
    ros::NodeHandle* node_; /**< node handle pointer*/
    std::vector<std::pair<std::string, std::string> > attachments_; /**< objects the agents have in hand*/
    std::vector<std::string> manipulableObjects_; /**< list of manipulable objects*/
    std::vector<std::string> supportObjects_; /**< list of support objects*/
    std::vector<std::string> containerObjects_; /**< list of container objects*/
    std::string humanHand_; /**< name of the human hand*/
    ros::Publisher previous_pub_; /**< publisher of previous actions*/
    ros::Publisher current_pub_; /**< publisher of humans current actions*/

};

#endif // HUMANMONITOR_H

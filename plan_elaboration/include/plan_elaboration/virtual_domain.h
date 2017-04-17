#ifndef VIRTUAL_DOMAIN_H
#define VIRTUAL_DOMAIN_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "toaster_msgs/Fact.h"

class VirtualDomain
{
public:
    VirtualDomain(ros::NodeHandle* node);
    ~VirtualDomain() {};
    virtual std::vector<toaster_msgs::Fact> computeSpecificFacts(std::vector<toaster_msgs::Fact> facts) = 0;
    std::string goal_; /**< current goal corresponding to the domain*/
    std::string agentLocked_; /**< agent locked for evaluation*/
    std::string objectLocked_; /**< object locked for evaluation*/
    std::string agentForgive_; /**< agent locked for forgiven actions*/
    std::string objectForgive_; /**< object locked for forgiven actions*/
    bool isHighLevelDomain_;  /**< Indicate if the domain is build with hugh level concepts*/

protected:
    ros::NodeHandle* node_; /**< Node handle*/
    std::string robotName_; /**< name of the robot */

    std::vector<toaster_msgs::Fact> computeLockedFacts(std::vector<toaster_msgs::Fact> facts);
    std::vector<toaster_msgs::Fact> computeForgiveFacts(std::vector<toaster_msgs::Fact> facts);
};

#endif // VIRTUAL_DOMAIN_H

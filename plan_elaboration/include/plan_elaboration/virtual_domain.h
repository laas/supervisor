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
    std::string agentLocked_; /**< agent locked for evaluation*/
    std::string objectLocked_; /**< object locked for evaluation*/
    bool isHighLevelDomain_;  /**< Indicate if the domain is build with hugh level concepts*/

protected:
    ros::NodeHandle* node_; /**< Node handle*/

    std::vector<toaster_msgs::Fact> computeLockedFacts(std::vector<toaster_msgs::Fact> facts);
};

#endif // VIRTUAL_DOMAIN_H

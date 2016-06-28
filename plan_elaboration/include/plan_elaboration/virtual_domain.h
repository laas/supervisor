#ifndef VIRTUAL_DOMAIN_H
#define VIRTUAL_DOMAIN_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "toaster_msgs/Fact.h"

using namespace std;

class VirtualDomain
{
public:
    VirtualDomain() {};
    ~VirtualDomain() {};
    virtual vector<toaster_msgs::Fact> computeSpecificFacts(vector<toaster_msgs::Fact> facts) = 0;
    vector<toaster_msgs::Fact> computeLockedFacts(vector<toaster_msgs::Fact> facts);
    string agentLocked_;
    string objectLocked_;
};

#endif // VIRTUAL_DOMAIN_H

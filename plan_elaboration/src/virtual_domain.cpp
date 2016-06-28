#include "plan_elaboration/virtual_domain.h"

/*
Add fact concerning the locked oject
*/
vector<toaster_msgs::Fact> VirtualDomain::computeLockedFacts(vector<toaster_msgs::Fact> facts){

    vector<toaster_msgs::Fact> toReturn = facts;
    toaster_msgs::Fact toAdd;
    toAdd.subjectId = objectLocked_;
    toAdd.property = "isLockedBy";
    toAdd.propertyType = "state";
    toAdd.targetId = agentLocked_;
    toReturn.push_back(toAdd);

    return toReturn;
}

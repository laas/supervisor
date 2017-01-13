#include "plan_elaboration/virtual_domain.h"

/**
 * \brief Constructor of the class
 * */
VirtualDomain::VirtualDomain(ros::NodeHandle* node)
{
    node_ = node;

}

/*
Add fact concerning the locked oject
*/
/**
 * \brief Compute facts concerning the locked oject
 * @param facts the initial set of facts
 * @return the initial set of facts + facts concerning the locked oject
 * */
std::vector<toaster_msgs::Fact> VirtualDomain::computeLockedFacts(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> toReturn = facts;
    toaster_msgs::Fact toAdd;
    toAdd.subjectId = objectLocked_;
    toAdd.property = "isLockedBy";
    toAdd.propertyType = "state";
    toAdd.targetId = agentLocked_;
    toReturn.push_back(toAdd);

    return toReturn;
}

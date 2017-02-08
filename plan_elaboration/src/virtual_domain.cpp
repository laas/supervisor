#include "plan_elaboration/virtual_domain.h"

/**
 * \brief Constructor of the class
 * */
VirtualDomain::VirtualDomain(ros::NodeHandle* node)
{
    node_ = node;

}

/**
 * \brief Compute facts concerning the locked oject
 * @param facts the initial set of facts
 * @return the initial set of facts + facts concerning the locked oject
 * */
std::vector<toaster_msgs::Fact> VirtualDomain::computeLockedFacts(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> toReturn = facts;
    toaster_msgs::Fact toAdd;
    toAdd.subjectId = agentLocked_;
    toAdd.property = "shouldPerform";
    toAdd.propertyType = "state";
    toAdd.targetId = objectLocked_;
    toReturn.push_back(toAdd);

    return toReturn;
}
/**
 * \brief Compute facts concerning the locked oject for forgive actions
 * @param facts the initial set of facts
 * @return the initial set of facts + facts concerning the locked oject for forgive actions
 * */
std::vector<toaster_msgs::Fact> VirtualDomain::computeForgiveFacts(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> toReturn = facts;
    toaster_msgs::Fact toAdd;
    toAdd.subjectId = objectForgive_;
    toAdd.property = "isForgivenFor";
    toAdd.propertyType = "state";
    toAdd.targetId = agentForgive_;
    toReturn.push_back(toAdd);

    return toReturn;
}


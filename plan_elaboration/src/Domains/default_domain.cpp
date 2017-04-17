#include "plan_elaboration/Domains/default_domain.h"

/**
 * \brief Constructor of the class
 * */
DefaultDomain::DefaultDomain(ros::NodeHandle* node) : VirtualDomain(node)
{
    isHighLevelDomain_ = false;
}

/**
 * \brief Default function to update the planning world state: do noting
 * @param facts the initial set of facts
 * @return the facts used for planning
 * */
std::vector<toaster_msgs::Fact> DefaultDomain::computeSpecificFacts(std::vector<toaster_msgs::Fact> facts){

    return facts;
}

#ifndef BLOCKS_DOMAIN_H
#define BLOCKS_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"


class BlocksDomain: public VirtualDomain
{
public:
    BlocksDomain(ros::NodeHandle* node);
    ~BlocksDomain() {};
    virtual std::vector<toaster_msgs::Fact> computeSpecificFacts(std::vector<toaster_msgs::Fact> facts);

private:
    std::map<std::string,bool> objectsStack_; /**< Map representing if each object is in the stack or not*/
    std::string areaStack_; /**< Name of the area where to build the stack*/

    std::vector<toaster_msgs::Fact> computeStack(std::vector<toaster_msgs::Fact> facts);
    std::vector<toaster_msgs::Fact> computeOwnAndAccess(std::vector<toaster_msgs::Fact> facts);
};

#endif // BLOCKS_DOMAIN_H

#ifndef DEFAULT_DOMAIN_H
#define DEFAULT_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"


class DefaultDomain: public VirtualDomain
{
public:
    DefaultDomain(ros::NodeHandle* node);
    ~DefaultDomain() {};
    virtual std::vector<toaster_msgs::Fact> computeSpecificFacts(std::vector<toaster_msgs::Fact> facts);
};

#endif // DEFAULT_DOMAIN_H

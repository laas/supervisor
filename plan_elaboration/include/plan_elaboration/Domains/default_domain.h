#ifndef DEFAULT_DOMAIN_H
#define DEFAULT_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"

using namespace std;

class DefaultDomain: public VirtualDomain
{
public:
    DefaultDomain();
    ~DefaultDomain() {};
    virtual vector<toaster_msgs::Fact> computeSpecificFacts(vector<toaster_msgs::Fact> facts);
};

#endif // DEFAULT_DOMAIN_H

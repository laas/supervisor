#ifndef BLOCKS_DOMAIN_H
#define BLOCKS_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"

using namespace std;

class BlocksDomain: public VirtualDomain
{
public:
    BlocksDomain();
    ~BlocksDomain() {};
    virtual vector<toaster_msgs::Fact> computeSpecificFacts(vector<toaster_msgs::Fact> facts);
};

#endif // BLOCKS_DOMAIN_H

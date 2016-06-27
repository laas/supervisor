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

private:
    map<string,bool> objectsStack_;
    string areaStack_;

    vector<toaster_msgs::Fact> computeStack(vector<toaster_msgs::Fact> facts);
    vector<toaster_msgs::Fact> computeOwnAndAccess(vector<toaster_msgs::Fact> facts);
};

#endif // BLOCKS_DOMAIN_H

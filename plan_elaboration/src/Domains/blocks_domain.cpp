#include "plan_elaboration/Domains/blocks_domain.h"

BlocksDomain::BlocksDomain() : VirtualDomain()
{
}

/*
Update the planning world state with blocks domain specific facts:
    - own
    - canAccess
    - isStack
    - isLockedBy
*/
vector<toaster_msgs::Fact> BlocksDomain::computeSpecificFacts(vector<toaster_msgs::Fact> facts){

    return facts;
}

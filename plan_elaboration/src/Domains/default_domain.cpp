#include "plan_elaboration/Domains/default_domain.h"

DefaultDomain::DefaultDomain() : VirtualDomain()
{
}

/*
Default function to update the planning world state: do noting
*/
vector<toaster_msgs::Fact> DefaultDomain::computeSpecificFacts(vector<toaster_msgs::Fact> facts){

    return facts;
}

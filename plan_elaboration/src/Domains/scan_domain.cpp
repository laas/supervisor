#include "plan_elaboration/Domains/scan_domain.h"

ScanDomain::ScanDomain() : VirtualDomain()
{
    areaScan_ = "scanAreaTABLE_4"; //TODO: to put in param
}

/*
Update the planning world state with blocks domain specific facts:
    - isInScanArea

*/
vector<toaster_msgs::Fact> ScanDomain::computeSpecificFacts(vector<toaster_msgs::Fact> facts){

    facts = computeScanArea(facts);

    return facts;
}

/*
Compute isInScanArea facts
*/
vector<toaster_msgs::Fact> ScanDomain::computeScanArea(vector<toaster_msgs::Fact> facts){


    vector<toaster_msgs::Fact> toReturn;

    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "IsInArea" && it->targetId == areaScan_){
            //we add the fact into the planning knowledge
            toaster_msgs::Fact toAdd;
            toAdd.subjectId = it->subjectId;
            toAdd.property = "isInScanArea";
            toAdd.propertyType = "state";
            toAdd.targetId = "true";
            toReturn.push_back(toAdd);
        }else if(it->property != "IsInArea"){
            //we remove all facts isInArea because they are not usefull for planning
            toReturn.push_back(*it);
        }
    }

    return toReturn;

}

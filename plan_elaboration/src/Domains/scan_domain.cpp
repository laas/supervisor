#include "plan_elaboration/Domains/scan_domain.h"

/**
 * \brief Constructor of the class
 * */
ScanDomain::ScanDomain(ros::NodeHandle* node) : VirtualDomain(node)
{
    node_->getParam("/plan_elaboration/domains/SCAN/areaScan", areaScan_);
    isHighLevelDomain_ = true;
}


/**
 * \brief Update the planning world state with scan domain specific facts:
    - isInScanArea
 * @param facts the initial set of facts
 * @return the facts used for planning
 * */
std::vector<toaster_msgs::Fact> ScanDomain::computeSpecificFacts(std::vector<toaster_msgs::Fact> facts){

    facts = computeScanArea(facts);

    facts = computeForgiveFacts(facts);

    facts = computeLockedFacts(facts);

    return facts;
}


/**
 * \brief Compute isInScanArea facts
 * @param facts the initial set of facts
 * @return the initial set of facts + isInScanArea facts
 * */
std::vector<toaster_msgs::Fact> ScanDomain::computeScanArea(std::vector<toaster_msgs::Fact> facts){


    std::vector<toaster_msgs::Fact> toReturn;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
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

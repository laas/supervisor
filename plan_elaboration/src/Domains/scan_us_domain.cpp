#include "plan_elaboration/Domains/scan_us_domain.h"

/**
 * \brief Constructor of the class
 * */
ScanUsDomain::ScanUsDomain(ros::NodeHandle* node) : VirtualDomain(node)
{
    node_->getParam("/supervisor/systemMode", systemMode_);
    if(systemMode_ == "new"){
        isHighLevelDomain_ = true;
    }else{
        isHighLevelDomain_ = false;
    }
}


/**
 * \brief Update the planning world state with scan domain specific facts:
    - isInScanArea
 * @param facts the initial set of facts
 * @return the facts used for planning
 * */
std::vector<toaster_msgs::Fact> ScanUsDomain::computeSpecificFacts(std::vector<toaster_msgs::Fact> facts){

    computeScanned(facts);
    facts = computeHasInHand(facts);

    facts = computeHasOn(facts);

    facts = computeForgiveFacts(facts);

    facts = computeLockedFacts(facts);

    facts = computeIsActivated(facts);

    return facts;
}


/**
 * \brief hasInRightHand and hasInLeftHand facts
 * @param facts the initial set of facts
 * @return the initial set of facts + hasInHands facts
 * */
std::vector<toaster_msgs::Fact> ScanUsDomain::computeHasInHand(std::vector<toaster_msgs::Fact> facts){


    std::vector<toaster_msgs::Fact> toReturn;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "isHoldBy"){
            if(objectsScanned_[it->subjectId] == true){
                toaster_msgs::Fact toAdd;
                toAdd.subjectId = it->targetId;
                toAdd.property = "hasInLeftHand";
                toAdd.propertyType = "state";
                toAdd.targetId = it->subjectId;
                toReturn.push_back(toAdd);
            }else{
                toaster_msgs::Fact toAdd;
                toAdd.subjectId = it->targetId;
                toAdd.property = "hasInRightHand";
                toAdd.propertyType = "state";
                toAdd.targetId = it->subjectId;
                toReturn.push_back(toAdd);
            }
        }else{
            //we remove hasInHand facts
            toReturn.push_back(*it);
        }
    }

    return toReturn;

}


/**
 * \brief hasInRightHand and hasInLeftHand facts
 * @param facts the initial set of facts
 * @return the initial set of facts + hasInHands facts
 * */
std::vector<toaster_msgs::Fact> ScanUsDomain::computeHasOn(std::vector<toaster_msgs::Fact> facts){


    std::vector<toaster_msgs::Fact> toReturn;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "isOn"){
            toaster_msgs::Fact toAdd;
            toAdd.subjectId = it->targetId;
            toAdd.property = "hasOn";
            toAdd.propertyType = "state";
            toAdd.targetId = it->subjectId;
            toReturn.push_back(toAdd);
        }
        toReturn.push_back(*it);
    }

    return toReturn;
}

/**
 * \brief fill the map for objects scanned
 * @param facts the initial set of facts
 * */
void ScanUsDomain::computeScanned(std::vector<toaster_msgs::Fact> facts){


    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "isScan"){
            if(it->targetId == "true"){
                objectsScanned_[it->subjectId] = true;
            }else{
                objectsScanned_[it->subjectId] = false;
            }
        }
    }
}

/**
 * \brief compute is activated facst
 * @param facts the initial set of facts
 * @retuen the initial set of facts + is Activated facts
 * */
std::vector<toaster_msgs::Fact> ScanUsDomain::computeIsActivated(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> toReturn = facts;

    toaster_msgs::Fact toAdd;
    toAdd.subjectId = robotName_;
    toAdd.property = "isActivated";
    toAdd.propertyType = "state";
    toAdd.targetId = "true";
    toReturn.push_back(toAdd);
    toAdd.subjectId = "HERAKLES_HUMAN1";
    toReturn.push_back(toAdd);

    if(systemMode_ == "hold"){
        toAdd.targetId = "false";
    }else{
	ROS_WARN("system mode: %s", systemMode_.c_str());
    }
    toAdd.subjectId = "AGENTX";
    toReturn.push_back(toAdd);
    toAdd.subjectId = "AGENTX2";
    toReturn.push_back(toAdd);

    return toReturn;
}

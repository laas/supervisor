#include "plan_elaboration/Domains/blocks_domain.h"

/**
 * \brief Constructor of the class
 * */
BlocksDomain::BlocksDomain(ros::NodeHandle* node) : VirtualDomain(node)
{
    node_->getParam("/plan_elaboration/domains/SCAN/areaStack", areaStack_);
    isHighLevelDomain_ = true;
}


/**
 * \brief Update the planning world state with blocks domain specific facts:
    - own
    - canAccess
    - isStack
    - isLockedBy
 * @param facts the initial set of facts
 * @return the facts used for planning
 * */
std::vector<toaster_msgs::Fact> BlocksDomain::computeSpecificFacts(std::vector<toaster_msgs::Fact> facts){

    facts = computeStack(facts);

    facts = computeOwnAndAccess(facts);

    facts = computeLockedFacts(facts);

    return facts;
}

/**
 * \brief Compute isStack facts
 * @param facts the initial set of facts
 * @return the initial set of facts + isStack facts
 * */
std::vector<toaster_msgs::Fact> BlocksDomain::computeStack(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> toReturn;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "IsInArea" && it->targetId == areaStack_){
            //we update the map
            objectsStack_[it->subjectId] = true;
            //we add the fact into the planning knowledge
            toaster_msgs::Fact toAdd;
            toAdd.subjectId = it->subjectId;
            toAdd.property = "isStack";
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


/**
 * \brief Compute own and canAccess facts
 * @param facts the initial set of facts
 * @return the initial set of facts + own and canAccess facts
 * */
std::vector<toaster_msgs::Fact> BlocksDomain::computeOwnAndAccess(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> toReturn;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "isReachableBy"){
            if(!objectsStack_.count(it->subjectId)>0){
                //the object is not stack
                objectsStack_[it->subjectId] = false;
            }
            toaster_msgs::Fact toAdd;
            toAdd.subjectId = it->targetId;
            if(objectsStack_[it->subjectId]){
                toAdd.property = "canAccess";
            }else{
                toAdd.property = "own";
            }
            toAdd.propertyType = "state";
            toAdd.targetId = it->subjectId;
            toReturn.push_back(toAdd);
        }else{
            //we remove all facts isReachableBy because they are not usefull for planning
            toReturn.push_back(*it);
        }
    }

    return toReturn;
}

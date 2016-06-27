#include "plan_elaboration/Domains/blocks_domain.h"

BlocksDomain::BlocksDomain() : VirtualDomain()
{
    areaStack_ = "stackAreaTABLE_4"; //TODO: to put in param
}

/*
Update the planning world state with blocks domain specific facts:
    - own
    - canAccess
    - isStack
*/
vector<toaster_msgs::Fact> BlocksDomain::computeSpecificFacts(vector<toaster_msgs::Fact> facts){

    facts = computeStack(facts);

    facts = computeOwnAndAccess(facts);


    return facts;
}

/*
Compute isStack fact
*/
vector<toaster_msgs::Fact> BlocksDomain::computeStack(vector<toaster_msgs::Fact> facts){

    vector<toaster_msgs::Fact> toReturn;

    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        if(it->property == "IsInArea" && it->targetId == areaStack_){
            //we update the map
            objectsStack_[it->subjectId] = true;
            //we add the fact into the planning knowledge
            toaster_msgs::Fact toAdd;
            toAdd.subjectId = it->subjectId;
            toAdd.property = "isStack";
            toAdd.propertyType = "state";
            toAdd.targetId = "false";
            toReturn.push_back(toAdd);
        }else if(it->property != "IsInArea"){
            //we remove all facts isInArea because they are not usefull for planning
            toReturn.push_back(*it);
        }
    }

    return toReturn;
}

/*
Compute own and canAccess facts
*/
vector<toaster_msgs::Fact> BlocksDomain::computeOwnAndAccess(vector<toaster_msgs::Fact> facts){

    vector<toaster_msgs::Fact> toReturn;

    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
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

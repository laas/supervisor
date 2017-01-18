#include "plan_elaboration/plan_elaboration.h"

/**
 * \brief Constructor of the class
 * */
PlanElaboration::PlanElaboration(ros::NodeHandle* node)
{
    currentGoal_ = "NONE";
    node_ = node;
    node_->getParam("/robot/name", robotName_);;
    node_->getParam("/supervisor/AgentX", agentX_);
    node_->getParam("/supervisor/Omni", omni_);
    node_->getParam("/supervisor/mainPartner", mainPartner_);
    node_->getParam("/plan_elaboration/Agents", agentList_);

    client_db_execute_  = node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    client_db_set_ = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
    client_hatp_ = node_->serviceClient<hatp_msgs::PlanningRequest>("hatp/planner");
    client_ask_ = node_->serviceClient<supervisor_msgs::Ask>("dialogue/ask");

}

/**
 * \brief Fill the highLevelNames map from param
 * */
void PlanElaboration::fillHighLevelNames(){

    //we retrieve the list objects from param and fill the map with their high level name
    std::vector<std::string> objects;
    node_->getParam("/entities/objects", objects);
    for(std::vector<std::string>::iterator it = objects.begin(); it != objects.end(); it++){
        std::string topic = "/highLevelName/" + *it;
        std::string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for supports
    std::vector<std::string> support;
    node_->getParam("/entities/supports", support);
    for(std::vector<std::string>::iterator it = support.begin(); it != support.end(); it++){
        std::string topic = "/highLevelName/" + *it;
        std::string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
    //same for containers
    std::vector<std::string> container;
    node_->getParam("/entities/containers", container);
    for(std::vector<std::string>::iterator it = container.begin(); it != container.end(); it++){
        std::string topic = "/highLevelName/" + *it;
        std::string highLevelName;
        if(node_->hasParam(topic)){
            node_->getParam(topic, highLevelName);
        }else{
            highLevelName = *it;
        }
        highLevelNames_[*it] = highLevelName;
    }
}


/**
 * \brief Initialize the domain considering the current goal
 * @param goal the current goal of the robot
 * */
VirtualDomain* PlanElaboration::initializeDomain(std::string goal){

    VirtualDomain* dom = NULL;

    if(goal == "BLOCKS"){
        dom = new BlocksDomain(node_);
    }else if(goal == "SCAN"){
        dom = new ScanDomain(node_);
    }else{
        dom = new DefaultDomain(node_);
    }

    return dom;
}

/**
 * \brief Find a plan for the current goal
 * @return bool for the success and the plan found if any
 * */
std::pair<bool, supervisor_msgs::SharedPlan> PlanElaboration::findPlan(){

    std::pair<bool, supervisor_msgs::SharedPlan> answer;
    dom_ = initializeDomain(currentGoal_);

    //Ask HATP a plan
    bool needPlan = true;
    while(needPlan){
        //Fill the planning database
        setPlanningTable(robotName_);
        std::pair<bool, hatp_msgs::Plan> hatpPlan = GetHATPPlan();
        if(hatpPlan.first){
            //Check feasability
            bool feasible = checkFeasible(hatpPlan.second);
            if(feasible){
                needPlan = false;
                //Ask plan for partner + check feasability: desactivated for now
                /*std::vector<std::string> goalActors;
                std::string actorsTopic = "goal_manager/goals/" + currentGoal_ + "_actors";
                node_->getParam(actorsTopic, goalActors);
                for(vector<string>::iterator it = goalActors.begin(); it != goalActors.end(); it++){
                    if(*it != robotName_){
                        setPlanningTable(*it);
                        pair<bool, hatp_msgs::Plan> hatpPartnerPlan = GetHATPPlan();
                        if(hatpPartnerPlan.first){
                            checkDivergentBelief(hatpPartnerPlan.second, *it);
                        }
                    }
                }*/

                //Apply post-process
                answer.second = convertPlan(hatpPlan.second);
                answer.first = true;
            }else{
                bool solve = solveFeasible(hatpPlan.second);
                if(!solve){
                    answer.first = false;
                    needPlan = false;
                }
                //else a new info has been given, we start again the proces of looking for a plan
            }
        }else{
            needPlan = false;
            answer.first = false;
        }
    }

    dom_->objectLocked_ = "NONE";


    return answer;
}



/**
 * \brief Set an answer from an SQL request in the database into facts
 * @param answer the SQL answer
 * @return a list of facts
 * */
std::vector<toaster_msgs::Fact> PlanElaboration::setAnswerIntoFacts(std::vector<std::string> answer){

    std::vector<toaster_msgs::Fact> result;

    int i = 0;
    toaster_msgs::Fact fact;
    for(std::vector<std::string>::iterator it = answer.begin(); it != answer.end(); it++){
        if(i == 0){
            fact.subjectId = *it;
            i++;
        }else if(i == 1){
            fact.property = *it;
            fact.propertyType = "state";
            i++;
        }else if(i == 2){
            fact.targetId = *it;
            result.push_back(fact);
            i = 0;
        }
    }

    return result;
}


/**
 * \brief Set facts into the planning table of the database (with AgentX capabilities)
 * @param agent the agent table we take has base for the planning
 * */
void PlanElaboration::setPlanningTable(std::string agent){

    toaster_msgs::ExecuteDB srv_exec;
    toaster_msgs::SetInfoDB srv_set;

    //we get from param the facts we need for planning
    std::vector<std::string> planningFactNames;
    std::string planningFactTopic = "plan_elaboration/domains/" + currentGoal_ + "/facts";
    node_->getParam(planningFactTopic, planningFactNames);


    //we ask to the database these facts in the agent table
    std::string sqlOrder = "SELECT subject_id, predicate, target_id from fact_table_" + agent + " where predicate=";
    bool beg = true;
    for(std::vector<std::string>::iterator it = planningFactNames.begin(); it != planningFactNames.end(); it++){
        if(beg){
            beg = false;
        }else{
            sqlOrder = sqlOrder + " OR predicate=";
        }
        sqlOrder = sqlOrder + "'" + *it + "'";
    }
    srv_exec.request.command = "SQL";
    srv_exec.request.order = sqlOrder;
    if (client_db_execute_.call(srv_exec)){
        //we put the answer into facts
        std::vector<toaster_msgs::Fact> agentFacts = setAnswerIntoFacts(srv_exec.response.results);
        if(agent == robotName_){
            robotFacts_ = agentFacts;
        }
        //we add specific domains fact in the world state
        std::vector<toaster_msgs::Fact> updatedFacts = dom_->computeSpecificFacts(agentFacts);
        //we get the final list of facts containing agentX capabilities
        std::vector<toaster_msgs::Fact> finalFacts = computeAgentXFacts(updatedFacts);
        //we update the planning table with these facts
        srv_set.request.infoType = "RESET_PLANNING";
        srv_set.request.facts = finalFacts;
        if (!client_db_set_.call(srv_set)){
            ROS_ERROR("[plan_elaboration] Failed to call service database_manager/set_info");
        }
    }else{
        ROS_ERROR("[plan_elaboration] Failed to call service database_manager/execute");
    }

}

/**
 * \brief Say if a string is in a vector of string
 * @param list the vector
 * @param element the looking string
 * @return true if element is in list
 * */
bool PlanElaboration::isInVector(std::vector<std::string> list, std::string element){

    for(std::vector<std::string>::iterator it = list.begin(); it != list.end(); it++){
        if(*it == element)
            return true;
    }

    return false;
}

/**
 * \brief Compute AgentX facts
 * @param facts the initial list of facts
 * @return the initial facst + agentX facts
 * */
std::vector<toaster_msgs::Fact> PlanElaboration::computeAgentXFacts(std::vector<toaster_msgs::Fact> facts){

    std::vector<toaster_msgs::Fact> result;
    std::string topic, highLevelIt, highLevelIt2;
    bool highLevelName;

    for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        result.push_back(*it);
        if(isInVector(agentList_, it->subjectId) && it->targetId != dom_->objectLocked_){
            bool added = false;
            //if the other object has an high level name we look for other object with high level names
            highLevelIt = highLevelNames_[it->targetId];
            for(std::vector<toaster_msgs::Fact>::iterator it2 = it+1; it2 != facts.end(); it2++){
                if(it2->targetId != dom_->objectLocked_){
                    highLevelIt2 = highLevelNames_[it2->targetId];
                    if(highLevelIt2 == it2->targetId){
                        highLevelName = false;
                    }else{
                        highLevelName = true;
                    }
                    if(it2->property == it->property && highLevelIt2 == highLevelIt && isInVector(agentList_, it2->subjectId) && it2->subjectId != it->subjectId){
                        result.push_back(*it2);
                        if(!added){
                            toaster_msgs::Fact toAdd;
                            toAdd.subjectId = agentX_;
                            toAdd.property = it->property;
                            toAdd.propertyType = "state";
                            toAdd.targetId = it->targetId;
                            result.push_back(toAdd);
                            //TODO remove when HATP ok
                            toAdd.subjectId = "AGENTX2";
                            result.push_back(toAdd);
                            added = true;
                        }
                        if(highLevelName){
                            toaster_msgs::Fact toAdd;
                            toAdd.subjectId = agentX_;
                            toAdd.property = it->property;
                            toAdd.propertyType = "state";
                            toAdd.targetId = it2->targetId;
                            result.push_back(toAdd);
                            toAdd.subjectId = "AGENTX2";
                            result.push_back(toAdd);
                        }
                        facts.erase(it2);
                        it2--;
                    }
                }
            }
        }else if(isInVector(agentList_, it->targetId) && it->subjectId != dom_->objectLocked_){
            bool added = false;
            //if the other object has an high level name we look for other objects with high level names
            highLevelIt = highLevelNames_[it->targetId];
            for(std::vector<toaster_msgs::Fact>::iterator it2 = it+1; it2 != facts.end(); it2++){
                if(it2->subjectId != dom_->objectLocked_){
                    highLevelIt2 = highLevelNames_[it2->targetId];
                    if(highLevelIt2 == it2->targetId){
                        highLevelName = false;
                    }else{
                        highLevelName = true;
                    }
                    if(it2->property == it->property && highLevelIt2 == highLevelIt && isInVector(agentList_, it2->targetId) && it2->targetId != it->targetId){
                        result.push_back(*it2);
                        if(!added){
                            toaster_msgs::Fact toAdd;
                            toAdd.subjectId = it->subjectId;
                            toAdd.property = it->property;
                            toAdd.propertyType = "state";
                            toAdd.targetId = agentX_;
                            result.push_back(toAdd);
                            toAdd.targetId = "AGENTX2";
                            result.push_back(toAdd);
                            added = true;
                        }
                        if(highLevelName){
                            toaster_msgs::Fact toAdd;
                            toAdd.subjectId = it2->subjectId;
                            toAdd.property = it->property;
                            toAdd.propertyType = "state";
                            toAdd.targetId = agentX_;
                            result.push_back(toAdd);
                            toAdd.targetId = "AGENTX2";
                            result.push_back(toAdd);
                        }
                        facts.erase(it2);
                        it2--;
                    }
                }
            }
        }
    }

    return result;
}


/**
 * \brief Ask a plan to HATP for the current goal
 * @return true if a plan is found the the plan if any
 * */
std::pair<bool, hatp_msgs::Plan> PlanElaboration::GetHATPPlan(){

    std::pair<bool, hatp_msgs::Plan> answer;
    hatp_msgs::PlanningRequest srv;

    //We look for the HATP method name to call
    std::string methodTopic = "plan_elaboration/domains/" + currentGoal_ + "/method";
    std::string methodName;
    node_->getParam(methodTopic, methodName);
    srv.request.request.task=methodName;

    //We look for the HATP parameters to add
    std::string paramsTopic = "plan_elaboration/domains/" + currentGoal_ + "/params";
    std::vector<std::string> params;
    node_->getParam(paramsTopic, params);
    for(std::vector<std::string>::iterator it = params.begin(); it != params.end(); it++){
       srv.request.request.parameters.push_back(*it);
    }

    //We ask a plan to HATP
    srv.request.request.type="plan";
    if(client_hatp_.call(srv)){
       if(srv.response.solution.report == "OK"){
          answer.first = true;
          answer.second = srv.response.solution;

       }else{
          answer.first = false;
      }
    }else{
        ROS_ERROR("[plan_elaboration] Failed to call service 'hatp/planner'");
        answer.first = false;
    }

    return answer;
}


/**
 * \brief Function which check if a given HATP plan is feasible (based on Omni agent action)
 * @param plan the plan to check
 * @return true if the plan is feasible
 * */
bool PlanElaboration::checkFeasible(hatp_msgs::Plan plan){

   for(std::vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          if(isInVector(it->agents, omni_)){
              return false;
          }
       }
    }

    return true;
}


/**
 * \brief Function which try to find a solution to a not feasible plan
 * @param plan the plan
 * @return true if a new info has been received
 * */
bool PlanElaboration::solveFeasible(hatp_msgs::Plan plan){

   for(std::vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          if(isInVector(it->agents, omni_)){
              std::vector<toaster_msgs::Fact> precs;// = getPrec(*it);
              toaster_msgs::Fact missingPrec;// = getMissingPrec(precs, robotFacts_);
              //if the fact concerns the omni agent, we replace omi by the receiver in the fact (we ask him the question)
              if(missingPrec.targetId == omni_){
                  missingPrec.targetId = mainPartner_;
              }
              if(missingPrec.subjectId == omni_){
                  missingPrec.subjectId = mainPartner_;
              }
              supervisor_msgs::Ask srv;
              srv.request.type = "FACT";
              srv.request.waitForAnswer = true;
              srv.request.receiver = mainPartner_;
              srv.request.fact = missingPrec;
              if(client_ask_.call(srv)){
                  if(srv.response.boolAnswer){
                      return true;
                  }else{
                      return false;
                  }
              }else{
                  ROS_ERROR("[plan_elaboration] Failed to call service dialogue_node/ask");
                  return false;
              }
          }
       }
    }

    return false;
}



/**
 * \brief Function which convert a plan from HATP to a supervisor plan and apply post process
 * @param plan the plan to convert
 * @return plan in the supervisor format
 * */
supervisor_msgs::SharedPlan PlanElaboration::convertPlan(hatp_msgs::Plan plan){

   supervisor_msgs::SharedPlan newPlan;
   newPlan.goal = currentGoal_;
   for(std::vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          //TODO: to remove when good HATP domain
          for(std::vector<std::string>::iterator ita = it->agents.begin(); ita != it->agents.end(); ita++){
               if(*ita == "AGENTX2"){
                   *ita = "AGENTX";
               }
          }
          supervisor_msgs::Action action;
          std::string nameTopic = "plan_elaboration/HATP_actions/" + it->name;
          node_->getParam(nameTopic, action.name);
          action.id = it->id;
          action.actors = it->agents;
          //we replace objects name by higher level name when needed
          if(dom_->isHighLevelDomain_){
            action.parameter_values = convertParam(it->parameters);
          }else{
            action.parameter_values = it->parameters;
          }
          //get the parameters keys from param
          std::string paramTopic = "highLevelActions/";
          paramTopic = paramTopic + action.name + "_param";
          node_->getParam(paramTopic, action.parameter_keys );
          if(action.parameter_values.size() != action.parameter_keys.size()){
              ROS_ERROR("[plan_elaboration] Incorrect action parameters");
          }
          newPlan.actions.push_back(action);
       }
    }
    for(std::vector<hatp_msgs::StreamNode>::iterator it = plan.streams.begin(); it != plan.streams.end(); it++){
       for(std::vector<unsigned int>::iterator itt = it->successors.begin(); itt != it->successors.end(); itt++){
          //TODO: remove useless link when HATP ok
          supervisor_msgs::Link link;
          link.origin = it->taskId;
          link.following = *itt;
          newPlan.links.push_back(link);
       }
    }

    return newPlan;
}

/**
 * \brief Function which takes parameters from HATP and replace objects name by higher level name when needed
 * @param params the parameters to convert
 * @return paramaters with needed high level names
 * */
std::vector<std::string> PlanElaboration::convertParam(std::vector<std::string> params){

    std::vector<std::string> result;

    bool flag = false;
    std::string tmpParam;
    //the first param is the name of the actor, we remove it
    /** @todo treat the case where more than one actor? */
    for(std::vector<std::string>::iterator it = params.begin() + 1; it != params.end(); it++){
        if(!flag){//the parameter is an object, keep it in memory
            tmpParam = *it;
            flag = true;
        }else{//the parameter is a flag, change the object in memory accordingly
            if(*it == "FTRUE"){
                result.push_back(highLevelNames_[tmpParam]);
            }else{
                result.push_back(tmpParam);
            }
            flag = false;
        }
    }

    return result;
}


/**
 * \brief Function which return the preconditions of a hatp task
 * @param task the task
 * @return the preconditions of the task
 * */
std::vector<toaster_msgs::Fact> PlanElaboration::getPrec(hatp_msgs::Task task){

    std::vector<toaster_msgs::Fact> prec;

    //we convert the task into an action
    supervisor_msgs::Action action;
    std::string nameTopic = "highLevelActions/";
    nameTopic = nameTopic + task.name;
    node_->getParam(nameTopic, action.name);
    action.actors = task.agents;
    if(dom_->isHighLevelDomain_){
      action.parameter_values = convertParam(task.parameters);
    }else{
      action.parameter_values = task.parameters;
    }
    std::string paramTopic = "highLevelActions/";
    paramTopic = paramTopic + action.name + "_param";
    node_->getParam(paramTopic, action.parameter_keys );
    if(action.parameter_values.size() != action.parameter_keys.size()){
        ROS_ERROR("[plan_elaboration] Incorrect action parameters");
        return prec;
    }

    //we get the preconditions of the action
    std::string precTopic = "highLevelActions/";
    precTopic = precTopic + action.name + "_prec";
    std::vector<std::string> precString;
    node_->getParam(precTopic, precString);
    //we transform the preconditions into facts
    for(std::vector<std::string>::iterator p = precString.begin(); p != precString.end(); p++){
        int beg = p->find(',');
        int end = p->find(',', beg+1);
        toaster_msgs::Fact fact;
        std::string subject = p->substr(0, beg);
        if(subject == "mainAgent"){
            fact.subjectId = action.actors[0];
        }else{
            for(int i = 0; i < action.parameter_values.size(); i++){
                   if(action.parameter_keys[i] == subject){
                       fact.subjectId = action.parameter_values[i];
                       break;
                   }
            }
        }
        fact.property = p->substr(beg+2, end - beg - 2);
        fact.propertyType = "state";
        std::string target = p->substr(end+2, p->size() - end - 2);
        if(target == "mainAgent"){
            fact.subjectId = action.actors[0];
        }else{
            for(int i = 0; i < action.parameter_values.size(); i++){
                   if(action.parameter_keys[i] == target){
                       fact.subjectId = action.parameter_values[i];
                       break;
                   }
            }
        }
        prec.push_back(fact);
    }

    return prec;
}


/**
 * \brief Find the missing fact into an agent knowledge
 * @param precs the facts to look
 * @param agentFacts the knowledge of the agent
 * @return the missing fact
 * */
toaster_msgs::Fact PlanElaboration::getMissingPrec(std::vector<toaster_msgs::Fact> precs, std::vector<toaster_msgs::Fact> agentFacts){

    toaster_msgs::Fact fact;

    for(std::vector<toaster_msgs::Fact>::iterator it = precs.begin(); it != precs.end(); it++){
        if(!factsAreIn(*it, agentFacts)){
            return *it;
        }
    }

    return fact;
}

/**
 * \brief Function which return true if a fact is in a agent knowledge
 * @param fact the fact to look
 * @param facts the knowledge of the agent
 * @return true if the fact is in a agent knowledge
 * */
bool PlanElaboration::factsAreIn(toaster_msgs::Fact fact, std::vector<toaster_msgs::Fact> facts){

    if(fact.subjectId == "NULL"){
        for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            if(it->property == fact.property && highLevelNames_[it->targetId] == highLevelNames_[fact.targetId]){
                return false;
            }
        }
    }else if(fact.targetId == "NULL"){
        for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            if(it->property == fact.property && highLevelNames_[it->subjectId] == highLevelNames_[fact.subjectId]){
                return false;
            }
        }
    }else{
        bool find = false;
        for(std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            if(it->property == fact.property && highLevelNames_[it->subjectId] == highLevelNames_[fact.subjectId] && highLevelNames_[it->targetId] == highLevelNames_[fact.targetId]){
                find = true;
                break;
            }
        }
        if(!find){
            return false;
        }
    }
    return true;
}


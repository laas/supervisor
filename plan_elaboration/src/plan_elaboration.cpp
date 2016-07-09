#include "plan_elaboration/plan_elaboration.h"

PlanElaboration::PlanElaboration(ros::NodeHandle* node)
{
    currentGoal_ = "NONE";
    needPlan_ = false;
    node_ = node;
    node_->getParam("/robot/name", robotName_);
    node_->getParam("/HATP/AgentX", agentX_);
    node_->getParam("/HATP/Omni", omni_);

    ros::ServiceClient client = node_->serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    supervisor_msgs::GetInfo srv;
    srv.request.info = "AGENTS";
    if (client.call(srv)){
       agentList_ = srv.response.agents;
    }else{
      ROS_ERROR("[state_machines] Failed to call service mental_state/get_all_agents");
    }
}

/*
New goal to execute
*/
void PlanElaboration::setGoal(string goal){

    currentGoal_ = goal;
    needPlan_ = true;
    dom_ = initializeDomain(currentGoal_);
    dom_->objectLocked_ = "NONE";

}

void PlanElaboration::checkPlan(){

    if(needPlan_){
        needPlan_ = false;
        ros::ServiceClient client = node_->serviceClient<supervisor_msgs::EndPlan>("goal_manager/end_goal");
        ros::ServiceClient clientCS = node_->serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");

        pair<bool, supervisor_msgs::Plan> plan = findPlan();
        if(plan.first){ //we send the plan to the MS and wait report
            supervisor_msgs::ChangeState serviceCS;
            serviceCS.request.type = "plan";
            serviceCS.request.state = "PROGRESS_SHARE";
            serviceCS.request.plan = plan.second;
            if(!clientCS.call(serviceCS)){
                ROS_ERROR("Failed to call service mental_state/change_state");
            }
        }else{ //if no plan found we send a report to the goal_manager
            supervisor_msgs::EndPlan srv;
            srv.request.report = false;
            if (!client.call(srv)){
                ROS_ERROR("[plan_elaboration] Failed to call service goal_manager/end_goal");
            }
        }
    }
}

/*
The current plan is over:
    @report: true if the current goal is achieved
*/
void PlanElaboration::endPlan(bool report){

    if(report){//the current goal is over
        currentGoal_ = "NONE";
    }else{
        needPlan_ = true;
    }

}

/*
Initialize the domain considering the current goal
*/
VirtualDomain* PlanElaboration::initializeDomain(string goal){

    VirtualDomain* dom = NULL;

    if(goal == "BLOCKS"){
        dom = new BlocksDomain();
    }else{
        dom = new DefaultDomain();
    }

    return dom;
}

/*
Find a plan with HATP, check its feasability and divergent belief, and apply post-process on it
*/
pair<bool, supervisor_msgs::Plan> PlanElaboration::findPlan(){

    pair<bool, supervisor_msgs::Plan> answer;

    //Ask HATP a plan
    bool needPlan = true;
    while(needPlan){
        //Fill the planning database
        setPlanningTable(robotName_);
        pair<bool, hatp_msgs::Plan> hatpPlan = GetHATPPlan();
        if(hatpPlan.first){
            //Check feasability
            vector<string> goalActors;
            string actorsTopic = "/goals/";
            actorsTopic = actorsTopic + currentGoal_ + "_actors";
            node_->getParam(actorsTopic, goalActors);
            for(vector<string>::iterator it = goalActors.begin(); it != goalActors.end(); it++){
                if(*it != robotName_){
                    goalPartner_  = *it;
                    break;
                }
            }
            string feasible = checkFeasible(hatpPlan.second);
            if(feasible == "ok"){
                needPlan = false;
                //Ask plan for partner + check feasability
                for(vector<string>::iterator it = goalActors.begin(); it != goalActors.end(); it++){
                    if(*it != robotName_){
                        setPlanningTable(*it);
                        pair<bool, hatp_msgs::Plan> hatpPartnerPlan = GetHATPPlan();
                        if(hatpPartnerPlan.first){
                            checkDivergentBelief(hatpPartnerPlan.second, *it);
                        }
                    }
                }

                //Apply post-process
                answer.second = convertPlan(hatpPlan.second);
                answer.first = true;
            }else if(feasible == "failed"){
                answer.first = false;
                needPlan = false;
            }
        }else{
            needPlan = false;
            answer.first = false;
        }
    }

    dom_->objectLocked_ = "NONE";

    return answer;
}


/*
Set facts into the planning table of the database (with AgentX capabilities)
 @agent: the agent table we take has base for the planning
*/
void PlanElaboration::setPlanningTable(string agent){

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    ros::ServiceClient client_set = node.serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
    toaster_msgs::ExecuteDB srv;
    toaster_msgs::SetInfoDB srv_set;

    //we get from param the facts we need for planning
    vector<string> planningFactNames;
    string planningFactTopic = "/HATP_domains/";
    planningFactTopic = planningFactTopic + currentGoal_ + "_facts";
    node_->getParam(planningFactTopic, planningFactNames);


    //we ask to the database these facts in the agent table
    string sqlOrder = "SELECT subject_id, predicate, target_id from fact_table_";
    sqlOrder = sqlOrder + agent + " where predicate=";
    bool beg = true;
    for(vector<string>::iterator it = planningFactNames.begin(); it != planningFactNames.end(); it++){
        if(beg){
            beg = false;
        }else{
            sqlOrder = sqlOrder + " OR predicate=";
        }
        sqlOrder = sqlOrder + "'" + *it + "'";
    }
    srv.request.command = "SQL";
    srv.request.order = sqlOrder;
    if (client.call(srv)){
        //we put the answer into facts
        vector<toaster_msgs::Fact> agentFacts = setAnswerIntoFacts(srv.response.results);
        if(agent == robotName_){
            robotFacts_ = agentFacts;
        }else{
            curAgentFacts_ = agentFacts;
        }
        //we add specific domains fact in the world state
        vector<toaster_msgs::Fact> updatedFacts = dom_->computeSpecificFacts(agentFacts);
        //we get the final list of facts containing agentX capabilities
        vector<toaster_msgs::Fact> finalFacts = computeAgentXFacts(updatedFacts);
        //we update the planning table with these facts
        srv_set.request.infoType = "RESET_PLANNING";
        srv_set.request.facts = finalFacts;
        if (!client_set.call(srv_set)){
            ROS_ERROR("[plan_elaboration] Failed to call service database_manager/set_info");
        }
    }else{
        ROS_ERROR("[plan_elaboration] Failed to call service database_manager/execute");
    }


}

/*
Say if a string is in a vector of string
*/
bool PlanElaboration::isInVector(vector<string> list, string element){

    for(vector<string>::iterator it = list.begin(); it != list.end(); it++){
        if(*it == element)
            return true;
    }

    return false;
}

/*
Compute AgentX facts based on a list of facts
*/
vector<toaster_msgs::Fact> PlanElaboration::computeAgentXFacts(vector<toaster_msgs::Fact> facts){

    vector<toaster_msgs::Fact> result;
    string topic, highLevelIt, highLevelIt2;
    bool highLevelName;

    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        result.push_back(*it);
        if(isInVector(agentList_, it->subjectId) && it->targetId != dom_->objectLocked_){
            bool added = false;
            //if the other object has an high level name we look for other objects with high level names
            topic = "/highLevelName/" + it->targetId;
            if(node_->hasParam(topic)){
                node_->getParam(topic, highLevelIt);
            }else{
                highLevelIt = it->targetId;
            }
            for(vector<toaster_msgs::Fact>::iterator it2 = it+1; it2 != facts.end(); it2++){
                if(it2->targetId != dom_->objectLocked_){
                    topic = "/highLevelName/" + it2->targetId;
                    if(node_->hasParam(topic)){
                        node_->getParam(topic, highLevelIt2);
                        highLevelName = true;
                    }else{
                        highLevelIt2 = it2->targetId;
                        highLevelName = false;
                    }
                    if(it2->property == it->property && highLevelIt2 == highLevelIt && isInVector(agentList_, it2->subjectId)){
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
            topic = "/highLevelName/" + it->subjectId;
            if(node_->hasParam(topic)){
                node_->getParam(topic, highLevelIt);
            }else{
                highLevelIt = it->subjectId;
            }
            for(vector<toaster_msgs::Fact>::iterator it2 = it+1; it2 != facts.end(); it2++){
                if(it2->subjectId != dom_->objectLocked_){
                    topic = "/highLevelName/" + it2->subjectId;
                    if(node_->hasParam(topic)){
                        node_->getParam(topic, highLevelIt2);
                        highLevelName = true;
                    }else{
                        highLevelName = false;
                        highLevelIt2 = it2->subjectId;
                    }
                    if(it2->property == it->property && highLevelIt2 == highLevelIt && isInVector(agentList_, it2->targetId)){
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

/*
Set an answer from an SQL request in the database into facts
*/
vector<toaster_msgs::Fact> PlanElaboration::setAnswerIntoFacts(vector<string> answer){

    vector<toaster_msgs::Fact> result;

    int i = 0;
    toaster_msgs::Fact fact;
    for(vector<string>::iterator it = answer.begin(); it != answer.end(); it++){
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

/*
Ask a plan to HATP for the current goal
*/
pair<bool, hatp_msgs::Plan> PlanElaboration::GetHATPPlan(){

    pair<bool, hatp_msgs::Plan> answer;

    ros::ServiceClient client = node_->serviceClient<hatp_msgs::PlanningRequest>("Planner");
    hatp_msgs::PlanningRequest service;

    //We look for the HATP method name to call
    string methodTopic = "HATP_domains/";
    methodTopic = methodTopic + currentGoal_ + "_method";
    string methodName;
    node_->getParam(methodTopic, methodName);
    service.request.request.task=methodName;

    //We look for the HATP parameters to add
    string paramsTopic = "HATP_domains/";
    paramsTopic = paramsTopic + currentGoal_ + "_params";
    vector<string> params;
    node_->getParam(paramsTopic, params);
    for(vector<string>::iterator it = params.begin(); it != params.end(); it++){
       service.request.request.parameters.push_back(*it);
    }

    //We ask a plan to HATP
    service.request.request.type="plan";
    if(client.call(service)){
       if(service.response.solution.report == "OK"){
          answer.first = true;
          answer.second = service.response.solution;

       }else{
          answer.first = false;
      }
    }else{
        ROS_ERROR("[plan_elaboration] Failed to call service 'Planner'");
        answer.first = false;
    }

    return answer;
}

/*
Function which check if a given HATP plan is feasible (based on Omni agent action)
*/
string PlanElaboration::checkFeasible(hatp_msgs::Plan plan){

    ros::ServiceClient client = node_->serviceClient<supervisor_msgs::Ask>("dialogue_node/ask");
    ros::ServiceClient clientMS = node_->serviceClient<supervisor_msgs::InfoGiven>("mental_state/info_given");
    supervisor_msgs::InfoGiven srvMS;
    supervisor_msgs::Ask srv;
    srv.request.type = "FACT";
    srv.request.waitForAnswer = true;
    srv.request.receiver = goalPartner_;

   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          if(isInVector(it->agents, omni_)){
              vector<toaster_msgs::Fact> precs = getPrec(*it);
              toaster_msgs::Fact missingPrec = getMissingPrec(precs, robotFacts_);
              //if the fact concerns the omni agent, we replace omi by the receiver in the fact (ask we ask him the question)
              if(missingPrec.targetId == omni_){
                  missingPrec.targetId = goalPartner_;
              }
              if(missingPrec.subjectId == omni_){
                  missingPrec.subjectId = goalPartner_;
              }

              srv.request.fact = missingPrec;
              if(client.call(srv)){
                  if(srv.response.boolAnswer){
                      srvMS.request.infoType = "fact";
                      srvMS.request.fact = missingPrec;
                      srvMS.request.receiver = robotName_;
                      srvMS.request.sender = goalPartner_;
                      ROS_INFO("receiver: %s sender: %s %s", robotName_.c_str(), goalPartner_.c_str());
                      if (!clientMS.call(srvMS)){
                         ROS_ERROR("[plan_elaboration] Failed to call service mental_states/info_given");
                         return "failed";
                      }
                      return "new_info";
                  }else{
                      return "failed";
                  }
              }else{
                  ROS_ERROR("[plan_elaboration] Failed to call service dialogue_node/ask");
                  return "failed";
              }
          }
       }
    }

    return "ok";
}

/*
Function which inform about divergent belief of a given HATP plan (based on Omni agent action)
*/
void PlanElaboration::checkDivergentBelief(hatp_msgs::Plan plan, string agent){

   ros::ServiceClient client = node_->serviceClient<supervisor_msgs::GiveInfo>("dialogue_node/give_info");
   supervisor_msgs::GiveInfo srv;
   srv.request.type = "FACT";
   srv.request.isTrue = true;
   srv.request.receiver = agent;

   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          if(isInVector(it->agents, omni_)){
              vector<toaster_msgs::Fact> precs = getPrec(*it);
              for(vector<toaster_msgs::Fact>::iterator it = precs.begin(); it != precs.end(); it++){
                  if(!factsIsIn(*it, curAgentFacts_)){
                      srv.request.fact = *it;
                      if(!client.call(srv)){
                          ROS_ERROR("[plan_elaboration] Failed to call service dialogue_node/give_info");
                      }
                  }
              }

          }
       }
    }
}

/*
Function which convert a plan from HATP to a supervisor plan and apply post process
*/
supervisor_msgs::Plan PlanElaboration::convertPlan(hatp_msgs::Plan plan){

   supervisor_msgs::Plan newPlan;
   newPlan.goal = currentGoal_;
   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          //TODO: to remove when good HATP domain
          for(vector<string>::iterator ita = it->agents.begin(); ita != it->agents.end(); ita++){
               if(*ita == "AGENTX2"){
                   *ita = "AGENTX";
               }
          }

          supervisor_msgs::Action action;
          string nameTopic = "HATP_actions/";
          nameTopic = nameTopic + it->name;
          node_->getParam(nameTopic, action.name);
          action.id = it->id;
          action.actors = it->agents;
          //we replace objects name by higher level name when needed
          action.parameters = convertParam(it->parameters);
          newPlan.actions.push_back(action);
       }
    }
    for(vector<hatp_msgs::StreamNode>::iterator it = plan.streams.begin(); it != plan.streams.end(); it++){
       for(vector<unsigned int>::iterator itt = it->successors.begin(); itt != it->successors.end(); itt++){
          //TODO: remove useless link when HATP ok
          supervisor_msgs::Link link;
          link.origin = it->taskId;
          link.following = *itt;
          newPlan.links.push_back(link);
       }
    }

    return newPlan;
}

/*
Function which takes parameters from HATP and replace objects name by higher level name when needed
*/
vector<string> PlanElaboration::convertParam(vector<string> params){

    vector<string> result;

    //the first param is the name of the actor, we remove it
    //TODO: treat the case where more than one actor?
    bool flag = false;
    string tmpParam;
    for(vector<string>::iterator it = params.begin() + 1; it != params.end(); it++){
        if(!flag){//the parameter is an object, keep it in memory
            tmpParam = *it;
            flag = true;
        }else{//the parameter is a flag, change the object in memory accordingly
            if(*it == "FTRUE"){
                string topic = "/highLevelName/";
                topic = topic + tmpParam;
                if(node_->hasParam(topic)){
                    string higlLevelName;
                    node_->getParam(topic, higlLevelName);
                    result.push_back(higlLevelName);
                }else{
                    //if no high level name in param we consider the object name to be high level
                    result.push_back(tmpParam);
                }
            }else{
                result.push_back(tmpParam);
            }
            flag = false;
        }
    }

    return result;
}

/*
Function which return the preconditions of a hatp task
*/
vector<toaster_msgs::Fact> PlanElaboration::getPrec(hatp_msgs::Task task){

    vector<toaster_msgs::Fact> prec;

    //we convert the task into an action
    supervisor_msgs::Action action;
    string nameTopic = "HATP_actions/";
    nameTopic = nameTopic + task.name;
    node_->getParam(nameTopic, action.name);
    action.actors = task.agents;
    action.parameters = convertParam(task.parameters);

    //we get the high level parameters of the action
    string paramTopic = "highLevelActions/";
    paramTopic = paramTopic + action.name + "_param";
    vector<string> params;
    node_->getParam(paramTopic, params);
    //we match then with the action param
    map<string, string> highLevelNames;
    highLevelNames["NULL"] = "NULL";
    if(action.parameters.size() == params.size()){
        vector<string>::iterator it2 = action.parameters.begin();
        for(vector<string>::iterator it = params.begin(); it != params.end(); it++){
            highLevelNames[*it] = *it2;
            it2++;
        }
    }else{
        ROS_ERROR("[plan_elaboration] Incorrect number of parameters");
        return prec;
    }
    string agentTopic = "highLevelActions/";
    agentTopic = agentTopic + action.name + "_actors";
    vector<string> agents;
    node_->getParam(agentTopic, agents);
    if(action.actors.size() == agents.size()){
            vector<string>::iterator it2 = action.actors.begin();
            for(vector<string>::iterator it = agents.begin(); it != agents.end(); it++){
                highLevelNames[*it] = *it2;
                it2++;
            }
        }else{
            ROS_ERROR("[plan_elaboration] Incorrect number of actors");
            return prec;
        }
    //we get the preconditions from param
    string precTopic = "highLevelActions/";
    precTopic = precTopic + action.name + "_prec";
    vector<string> precString;
    node_->getParam(precTopic, precString);
    //we transform the preconditions into facts
    for(vector<string>::iterator p = precString.begin(); p != precString.end(); p++){
        int beg = p->find(',');
        int end = p->find(',', beg+1);
        toaster_msgs::Fact fact;
        fact.subjectId = highLevelNames[p->substr(0, beg)];
        fact.property = p->substr(beg+2, end - beg - 2);
        fact.propertyType = "state";
        fact.targetId = highLevelNames[p->substr(end+2, p->size() - end - 2)];
        prec.push_back(fact);
    }

    return prec;
}

/*
Find the missing fact into an agent knowledge
*/
toaster_msgs::Fact PlanElaboration::getMissingPrec(vector<toaster_msgs::Fact> precs, vector<toaster_msgs::Fact> agentFacts){

    toaster_msgs::Fact fact;

    for(vector<toaster_msgs::Fact>::iterator it = precs.begin(); it != precs.end(); it++){
        if(!factsIsIn(*it, agentFacts)){
            return *it;
        }
    }

    return fact;
}

/*
Function which return true a fact is in a agent knowledge
*/
bool PlanElaboration::factsIsIn(toaster_msgs::Fact fact, vector<toaster_msgs::Fact> facts){

    string topicTarget = "/highLevelName/";
    topicTarget = topicTarget + fact.targetId;
    string higlLevelNameTarget;
    if(node_->hasParam(topicTarget)){
        node_->getParam(topicTarget, higlLevelNameTarget);
    }else{
        higlLevelNameTarget = fact.targetId;
    }
    string topicSubject = "/highLevelName/";
    topicSubject = topicSubject + fact.subjectId;
    string higlLevelNameSubject;
    if(node_->hasParam(topicSubject)){
        node_->getParam(topicSubject, higlLevelNameSubject);
    }else{
        higlLevelNameSubject = fact.subjectId;
    }
    if(fact.subjectId == "NULL"){
        for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            string topic = "/highLevelName/";
            topic = topic + it->targetId;
            string higlLevelName;
            if(node_->hasParam(topic)){
                node_->getParam(topic, higlLevelName);
            }else{
                higlLevelName = it->targetId;
            }
            if(it->property == fact.property && higlLevelName == higlLevelNameTarget){
                return false;
            }
        }
    }else if(fact.targetId == "NULL"){
        for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            string topic = "/highLevelName/";
            topic = topic + it->subjectId;
            string higlLevelName;
            if(node_->hasParam(topic)){
                node_->getParam(topic, higlLevelName);
            }else{
                higlLevelName = it->subjectId;
            }
            if(it->property == fact.property && higlLevelName == higlLevelNameSubject){
                return false;
            }
        }
    }else{
        bool find = false;
        for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
            string topicS = "/highLevelName/";
            topicS = topicS + it->subjectId;
            string higlLevelNameS;
            if(node_->hasParam(topicS)){
                node_->getParam(topicS, higlLevelNameS);
            }else{
                higlLevelNameS = it->subjectId;
            }
            string topicT = "/highLevelName/";
            topicT = topicT + it->targetId;
            string higlLevelNameT;
            if(node_->hasParam(topicT)){
                node_->getParam(topicT, higlLevelNameT);
            }else{
                higlLevelNameT = it->targetId;
            }
            if(it->property == fact.property && higlLevelNameS == higlLevelNameSubject && higlLevelNameT == higlLevelNameTarget){
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

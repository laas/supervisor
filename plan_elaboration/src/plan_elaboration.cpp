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
Find a plan with HATP, check its feasability and divergent belief, and apply post-process on it
*/
pair<bool, supervisor_msgs::Plan> PlanElaboration::findPlan(){

    pair<bool, supervisor_msgs::Plan> answer;

    //Fill the planning database
    setPlanningTable(robotName_);
    
    //Ask HATP a plan
    bool needPlan = true;
    while(needPlan){
        pair<bool, hatp_msgs::Plan> hatpPlan = GetHATPPlan();
        if(hatpPlan.first){
            //Check feasability
            string feasible = checkFeasible(hatpPlan.second);
            if(feasible == "ok"){
                needPlan = false;
                //Ask plan for partner + check feasability
                vector<string> goalActors;
                string actorsTopic = "/goals/";
                actorsTopic = actorsTopic + currentGoal_ + "_actors";
                node_->getParam(actorsTopic, goalActors);
                for(vector<string>::iterator it = goalActors.begin(); it != goalActors.end(); it++){
                    if(*it != robotName_){
                        setPlanningTable(*it);
                        pair<bool, hatp_msgs::Plan> hatpPartnerPlan = GetHATPPlan();
                        if(hatpPartnerPlan.first){
                            checkDivergentBelief(hatpPartnerPlan.second);
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


    return answer;
}


/*
Set facts into the planning table of the database (with AgentX capabilities)
 @agent: the agent table we take has base for the planning
*/
void PlanElaboration::setPlanningTable(string agent){

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<toaster_msgs::ExecuteDB>("database/execute");
    ros::ServiceClient client_set = node.serviceClient<toaster_msgs::SetInfoDB>("database/set_info");
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
        //we get the final list of facts containing agentX capabilities
        vector<toaster_msgs::Fact> finalFacts = computeAgentXFacts(agentFacts);
        //we update the planning table with these facts
        srv_set.request.infoType = "RESET_PLANNING";
        srv_set.request.facts = finalFacts;
        if (!client_set.call(srv_set)){
            ROS_ERROR("[plan_elaboration] Failed to call service database/set_info");
        }
    }else{
        ROS_ERROR("[plan_elaboration] Failed to call service database/execute");
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

    for(vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++){
        result.push_back(*it);
        if(isInVector(agentList_, it->subjectId)){
            bool added = false;
            for(vector<toaster_msgs::Fact>::iterator it2 = it+1; it2 != facts.end(); it2++){
                if(it2->property == it->property && it2->targetId == it->targetId && isInVector(agentList_, it2->subjectId)){
                    result.push_back(*it2);
                    if(!added){
                        toaster_msgs::Fact toAdd;
                        toAdd.subjectId = agentX_;
                        toAdd.property = it->property;
                        toAdd.targetId = it->targetId;
                        result.push_back(toAdd);
                        added = true;
                    }
                    facts.erase(it2);
                    it2--;
                }
            }
        }else if(isInVector(agentList_, it->targetId)){
            bool added = false;
            for(vector<toaster_msgs::Fact>::iterator it2 = it+1; it2 != facts.end(); it2++){
                if(it2->property == it->property && it2->subjectId == it->subjectId && isInVector(agentList_, it2->targetId)){
                    result.push_back(*it2);
                    if(!added){
                        toaster_msgs::Fact toAdd;
                        toAdd.subjectId = it->subjectId;
                        toAdd.property = it->property;
                        toAdd.targetId = agentX_;
                        result.push_back(toAdd);
                        added = true;
                    }
                    facts.erase(it2);
                    it2--;
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

   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          if(isInVector(it->agents, omni_)){
              //TODO: ask preconditions
              //TODO: if new info return new_info
              return "failed";
          }
       }
    }

    return "ok";
}

/*
Function which inform about divergent belief of a given HATP plan (based on Omni agent action)
*/
void PlanElaboration::checkDivergentBelief(hatp_msgs::Plan plan){

   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          if(isInVector(it->agents, omni_)){
              //TODO: inform preconditions
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

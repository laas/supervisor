#include "plan_elaboration/plan_elaboration.h"

PlanElaboration::PlanElaboration(ros::NodeHandle* node)
{
    currentGoal_ = "NONE";
    needPlan_ = false;
    node_ = node;
    node_->getParam("/robot/name", robotName_);

}

/*
New goal to execute
*/
void PlanElaboration::setGoal(string goal){

    currentGoal_ = goal;
    needPlan_ = true;
    dom_ = initializeDomain(currentGoal_);

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

    if(goal == "SCAN"){
        dom = new ScanDomain();
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

    //Fill the planning database
    setPlanningTable();
    pair<bool, hatp_msgs::Plan> hatpPlan = GetHATPPlan();
    if(hatpPlan.first){
            //Apply post-process
            answer.second = convertPlan(hatpPlan.second);
            answer.first = true;
    }else{
        answer.first = false;
    }


    return answer;
}


/*
Set facts into the planning table of the database
*/
void PlanElaboration::setPlanningTable(){

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
    sqlOrder = sqlOrder + robotName_ + " where predicate=";
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
        //we add specific domains fact in the world state
        vector<toaster_msgs::Fact> updatedFacts = dom_->computeSpecificFacts(agentFacts);
        //we update the planning table with these facts
        srv_set.request.infoType = "RESET_PLANNING";
        srv_set.request.facts = updatedFacts;
        if (!client_set.call(srv_set)){
            ROS_ERROR("[plan_elaboration] Failed to call service database_manager/set_info");
        }
    }else{
        ROS_ERROR("[plan_elaboration] Failed to call service database_manager/execute");
    }


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
    service.request.request.type="first-plan";
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
Function which convert a plan from HATP to a supervisor plan and apply post process
*/
supervisor_msgs::Plan PlanElaboration::convertPlan(hatp_msgs::Plan plan){

   supervisor_msgs::Plan newPlan;
   newPlan.goal = currentGoal_;
   for(vector<hatp_msgs::Task>::iterator it = plan.tasks.begin(); it != plan.tasks.end(); it++){
      if(it->type){//the task is an action and not a method
          supervisor_msgs::Action action;
          string nameTopic = "HATP_actions/";
          nameTopic = nameTopic + it->name;
          node_->getParam(nameTopic, action.name);
          action.id = it->id;
          action.actors = it->agents;
          //we remove the first parameter as it is the ame of the actor
          for(vector<string>::iterator itp = it->parameters.begin() +1; itp != it->parameters.end(); itp++){
              action.parameters.push_back(*itp);
          }
          newPlan.actions.push_back(action);
       }
    }
    for(vector<hatp_msgs::StreamNode>::iterator it = plan.streams.begin(); it != plan.streams.end(); it++){
       for(vector<unsigned int>::iterator itt = it->successors.begin(); itt != it->successors.end(); itt++){
          supervisor_msgs::Link link;
          link.origin = it->taskId;
          link.following = *itt;
          newPlan.links.push_back(link);
       }
    }

    return newPlan;
}

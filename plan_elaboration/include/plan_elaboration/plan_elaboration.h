#ifndef PLAN_ELABORATION_H

#define PLAN_ELABORATION_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "plan_elaboration/virtual_domain.h"
#include "plan_elaboration/Domains/default_domain.h"
#include "plan_elaboration/Domains/scan_domain.h"


#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/Plan.h"
#include "hatp_msgs/Plan.h"
#include "hatp_msgs/PlanningRequest.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/SetInfoDB.h"
#include "supervisor_msgs/GetInfo.h"
#include "supervisor_msgs/GiveInfo.h"
#include "supervisor_msgs/Ask.h"
#include "supervisor_msgs/Bool.h"
#include "supervisor_msgs/InfoGiven.h"
#include "supervisor_msgs/AgentList.h"


using namespace std;

class PlanElaboration
{
public:
    PlanElaboration(ros::NodeHandle* node);
    void setGoal(string goal);
    void endPlan(bool report);
    void checkPlan();
    VirtualDomain* dom_;

private:
    ros::NodeHandle* node_;
    string currentGoal_;
    bool needPlan_;
    string robotName_;


    pair<bool, supervisor_msgs::Plan> findPlan();
    pair<bool, hatp_msgs::Plan> GetHATPPlan();
    void setPlanningTable();
    vector<toaster_msgs::Fact> setAnswerIntoFacts(vector<string> answer);
    supervisor_msgs::Plan convertPlan(hatp_msgs::Plan plan);
    bool factsIsIn(toaster_msgs::Fact fact, vector<toaster_msgs::Fact> facts);
    VirtualDomain* initializeDomain(string goal);
};

#endif // PLAN_ELABORATION_H

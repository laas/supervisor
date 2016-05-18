#ifndef PLAN_ELABORATION_H

#define PLAN_ELABORATION_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/Plan.h"
#include "hatp_msgs/Plan.h"
#include "hatp_msgs/PlanningRequest.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/SetInfoDB.h"
#include "supervisor_msgs/GetInfo.h"

using namespace std;

class PlanElaboration
{
public:
    PlanElaboration(ros::NodeHandle* node);
    void setGoal(string goal);
    void endPlan(bool report);
    void checkPlan();

private:
    ros::NodeHandle* node_;
    string currentGoal_;
    bool needPlan_;
    string robotName_;
    string agentX_;
    string omni_;
    vector<string> agentList_;

    pair<bool, supervisor_msgs::Plan> findPlan();
    pair<bool, hatp_msgs::Plan> GetHATPPlan();
    void setPlanningTable(string agent);
    vector<toaster_msgs::Fact> setAnswerIntoFacts(vector<string> answer);
    vector<toaster_msgs::Fact> computeAgentXFacts(vector<toaster_msgs::Fact> facts);
    bool isInVectorFact(vector<toaster_msgs::Fact> list, toaster_msgs::Fact element);
    bool isInVector(vector<string> list, string element);
    string checkFeasible(hatp_msgs::Plan plan);
    supervisor_msgs::Plan convertPlan(hatp_msgs::Plan plan);
    void checkDivergentBelief(hatp_msgs::Plan plan);
    vector<string> convertParam(vector<string> params);
};

#endif // PLAN_ELABORATION_H

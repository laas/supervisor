#ifndef PLAN_ELABORATION_H

#define PLAN_ELABORATION_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "plan_elaboration/virtual_domain.h"
#include "plan_elaboration/Domains/default_domain.h"
#include "plan_elaboration/Domains/scan_domain.h"
#include "plan_elaboration/Domains/blocks_domain.h"

#include "std_srvs/Trigger.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/SetInfoDB.h"
#include "hatp_msgs/PlanningRequest.h"

#include "supervisor_msgs/GoalsList.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/SharedPlan.h"
#include "supervisor_msgs/String.h"
#include "supervisor_msgs/Ask.h"


class PlanElaboration
{
public:
    PlanElaboration(ros::NodeHandle* node);
    VirtualDomain* dom_; /**< Domain use for facts computation*/
    std::string currentGoal_; /**< current goal of the robot*/
    bool domainInitialized_; /**< flag to say if the domain is initialized for the current goal*/

    std::pair<bool, supervisor_msgs::SharedPlan> findPlan();
private:
    ros::NodeHandle* node_; /**< Node handle*/
    std::string robotName_; /**< name of the robot*/
    std::string agentX_; /**< HATP name of the x agent*/
    std::string omni_; /**< HATP name of the omni agent*/
    std::vector<std::string> agentList_; /**< name of all agents considered for planning*/
    std::string mainPartner_; /**< name of the main partner considered for planning*/
    std::vector<toaster_msgs::Fact> robotFacts_; /**< world state from the robot point of view*/
    std::map<std::string, std::string> highLevelNames_; /**< map for higl level names objects*/
    std::map<std::string, std::vector<std::string> > highLevelRefinment_; /**< possible refinment for high level names*/
    int planId_; /**< previous given id for a new plan*/
    int nbMaxTry_; /**< nb max try to get a plan*/

    ros::ServiceClient client_db_execute_; /**< client for the database execute service*/
    ros::ServiceClient client_db_set_; /**< client for the database set info service*/
    ros::ServiceClient client_hatp_; /**< client for the hatp planning service*/
    ros::ServiceClient client_ask_; /**< client for the ask service*/

    void fillHighLevelNames();
    VirtualDomain* initializeDomain(std::string goal);
    std::vector<toaster_msgs::Fact> setAnswerIntoFacts(std::vector<std::string> answer);
    void setPlanningTable(std::string agent);
    bool isInVector(std::vector<std::string> list, std::string element);
    std::vector<toaster_msgs::Fact> computeAgentXFacts(std::vector<toaster_msgs::Fact> facts);
    std::pair<bool, hatp_msgs::Plan> GetHATPPlan();
    bool checkFeasible(hatp_msgs::Plan plan);
    bool solveFeasible(hatp_msgs::Plan plan);
    supervisor_msgs::SharedPlan convertPlan(hatp_msgs::Plan plan);
    std::vector<std::string> convertParam(std::vector<std::string> params);
    std::vector<toaster_msgs::Fact> getPrec(hatp_msgs::Task task);
    toaster_msgs::Fact getMissingPrec(std::vector<toaster_msgs::Fact> precs, std::vector<toaster_msgs::Fact> agentFacts);
    bool factsAreIn(toaster_msgs::Fact fact, std::vector<toaster_msgs::Fact> facts);

};

#endif // PLAN_ELABORATION_H

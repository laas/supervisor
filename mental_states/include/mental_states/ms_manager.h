#ifndef MS_MANAGER_H

#define MS_MANAGER_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "boost/algorithm/string.hpp"

#include "toaster_msgs/DatabaseTables.h"
#include "toaster_msgs/SetInfoDB.h"
#include "toaster_msgs/ExecuteDB.h"

#include "supervisor_msgs/MentalStatesList.h"
#include "supervisor_msgs/SharedPlan.h"
#include "supervisor_msgs/GoalsList.h"
#include "supervisor_msgs/ActionsList.h"
#include "supervisor_msgs/Info.h"


class MsManager
{
public:
    MsManager(ros::NodeHandle* node);
    void initMentalStates();
    bool checkEffects();
    bool checkPrecs();
    void checkGoals();
    supervisor_msgs::Action addPrecsAndEffects(supervisor_msgs::Action action);
    bool canSee(std::string agent, std::string target);
    supervisor_msgs::Action isInList(supervisor_msgs::Action action, std::vector<supervisor_msgs::Action> actions);
    void addEffects(supervisor_msgs::Action action, std::string agent);
    void addRmFactToAgent(toaster_msgs::Fact fact, std::string agent, bool add);
    bool isInList(std::vector<std::string> list, std::string element);
    bool isVisibleBy(std::string target, std::string agent);

    std::string robotName_; /**< name of the robot*/
    std::string agentX_; /**< HATP name of the x agent*/
    std::map<std::string, std::string> highLevelNames_; /**< map for higl level names objects*/
    std::map<std::string, std::vector<std::string> > highLevelRefinment_; /**< possible refinment for high level names*/
    std::vector<std::string> agents_; /**< the agents for who we compute mental states*/
    std::vector<supervisor_msgs::MentalState> msList_; /**< the mental states of the agents*/
    supervisor_msgs::Action currentRobotAction_; /**< the current robot action*/
    supervisor_msgs::SharedPlan currentPlan_; /**< the current plan for the joint action*/
    std::string currentRobotGoal_; /**< the current goal of the robot*/
    std::vector<toaster_msgs::DatabaseTable> agentsTable_; /**< the knowledge of the agents from the database*/
private:
    ros::NodeHandle* node_; /**< Node handle*/
    ros::ServiceClient client_db_ ; /**< Client for the set info service of the database*/
    ros::ServiceClient client_execute_db_ ; /**< Client for the set info service of the database*/
    std::vector<std::string> nonObservableFacts_; /**< list of non observable facts*/

    void fillHighLevelNames();
    bool areFactsInTable(std::vector<toaster_msgs::Fact> facts, std::string agent, bool highLevel);

};

#endif // MS_MANAGER_H

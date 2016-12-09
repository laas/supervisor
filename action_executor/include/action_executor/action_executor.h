#ifndef ACTIONEXECUTOR_H
#define ACTIONEXECUTOR_H

#include "action_executor/connector.h"
#include "action_executor/virtual_action.h"

#include "action_executor/Actions/pick.h"
#include "action_executor/Actions/place.h"
#include "action_executor/Actions/placeReachable.h"
#include "action_executor/Actions/drop.h"
#include "action_executor/Actions/scan.h"
#include "action_executor/Actions/moveTo.h"
#include "action_executor/Actions/pickAndPlace.h"
#include "action_executor/Actions/pickAndPlaceReachable.h"
#include "action_executor/Actions/pickAndDrop.h"


typedef actionlib::SimpleActionServer<supervisor_msgs::ActionExecutorAction> Server;

class ActionExecutor{
public:
    ActionExecutor(std::string name, ros::NodeHandle* node);
    Connector connector_; /**< Connector structure which regroups usefull information*/
protected:
    supervisor_msgs::ActionExecutorFeedback feedback_; /**< feedback of the action*/
    supervisor_msgs::ActionExecutorResult result_; /**< result of the action*/
    Server action_server_; /**< action server*/

private:
    void execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal);
    VirtualAction* initializeAction(supervisor_msgs::Action action);
    void initHighLevelNames();

};

#endif // ACTIONEXECUTOR_H

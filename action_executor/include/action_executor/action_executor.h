#ifndef ACTIONEXECUTOR_H
#define ACTIONEXECUTOR_H

#include "action_executor/connector.h"
#include "action_executor/virtual_action.h"

#include "action_executor/Actions/pick.h"
#include "action_executor/Actions/place.h"

typedef actionlib::SimpleActionServer<supervisor_msgs::ActionExecutorAction> Server;

class ActionExecutor{
public:
    ActionExecutor(std::string name, ros::NodeHandle* node);
    Connector connector_; /**< Connector structure which regroups usefull information*/
    bool isActing_; /**< Flag indicating if the robot is executing an action*/
    supervisor_msgs::Action currentAction_; /**< Current action executed by the robot*/
protected:
    supervisor_msgs::ActionExecutorFeedback feedback_; /**< feedback of the action*/
    supervisor_msgs::ActionExecutorResult result_; /**< result of the action*/
    Server action_server_; /**< action server*/

private:
    void execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal);
    VirtualAction* initializeAction(supervisor_msgs::Action action);

};

#endif // ACTIONEXECUTOR_H

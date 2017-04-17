/** @file mainpage.h
* @brief Main page of the robot decision doxygen documentation
*
*/
/** \mainpage Robot Decision
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows the robot to decide which action to perform and when
*
* \section Suscribed topics
*   - supervisor/todo_actions: actions todo for all agents (supervisor_msgs/ActionsList)
*   - mental_states/mental_states: mental states of humans (supervisor_msgs/MentalStatesList)
*   - goal_manager/goalsList: goals of the robot (supervisor_msgs/GoalsList)
*   - area_manager/factList: toaster facts concerning areas (toaster_mgs/FactList)
*   - plan_elaboration/plan: current plan of the robot (supervisor_msgs/Plan)
*   - supervisor/previous_actions: previous actions of all agents (supervisor_msgs/ActionsList)
*
* \section Services provided
*   - robot_decision/stop: stop the current action of the robot if any (std_srvs/Empty)
*
* \section Action server needed
*   - supevisor/action_executor: allow the robot to execute an action (supervisor_msgs/ActionExecutor)
*
* \section Parameters
* - robot_decision/mode: the mode chosen for action attribution (negotiation or adaptation)
* - robot_decision/timeAdaptation: time to wait for the adaptation mode
* - robot_decision/timeWaitHuman: time to wait for an action of the human
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
*
* Run dependencies
* - Toaster
*/

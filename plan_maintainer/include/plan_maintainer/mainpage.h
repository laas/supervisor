/** @file mainpage.h
* @brief Main page of the plan maintainer doxygen documentation
*
*/
/** \mainpage Plan Maintainer
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to maintain the current plan of the robot
*
* \section Suscribed topics
*   - plan_elaboration/plan: current plan of the robot (supervisor_msgs/Plan)
*   - action_executor/current_robot_action: current action of the robot (supevisor_msgs/Action)
*   - supervisor/previous_actions: previous actions of all agents (supervisor_msgs/ActionsList)
*
* \section Published topics
*   - plan_maintainer/todo_actions: actions todo for all agents (supervisor_msgs/ActionsList)
*   - plan_maintainer/planned_actions: planned actions for all agents (supervisor_msgs/ActionsList)
*
* \section Services provided
*   - plan_maintainer/stop

*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
*
* Run dependencies
* - Toaster
*/

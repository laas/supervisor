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
*   - supervisor/todo_actions
*
* * \section Services provided
*   - robot_decision/stop
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

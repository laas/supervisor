/** @file mainpage.h
* @brief Main page of the simple head manager doxygen documentation
*
*/
/** \mainpage Simple Head Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to manage the head of the robot (in a simple way).
* It gives priority to:
*   - the speaking target if the robot is speaking (and not executing an action which require head focus)
*   - the human action target if a human performs an action (and not executing an action which require head focus)
*   - the robot action target if the robot is acting
*   - a default target
*
* \section Suscribed topics
*   - action_executor/current_robot_action: current action of the robot (supervisor_msgs/Action)
*   - human_monitor/current_humans_action: current actions of the humans (supervisor_msgs/ActionsList)
*   - supervisor/previous_actions: previous actions of all agents (supervisor_msgs/ActionsList)
*   - dialogue_node/isSpeaking: true if the robot is speaking (std_msgs/Bool)
*
* \section Published topics
*   - simple_head_manager/focus: where the robot is looking (std_msgs/String)
*
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
*
* Run dependencies
* - Toaster
*/

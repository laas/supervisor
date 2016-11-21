/** @file mainpage.h
* @brief Main page of the action executor doxygen documentation
*
*/
/** \mainpage Data Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows the robot to execute high level actions.
* The actions currently implemented are:
*   - Pick
* For action the node:
*   - checks its precondition
*   - plans the necessary trajectories
*   - executes the action
*   - aplies/checks the post conditions
*
* \section Suscribed topics
*  - /human_monitor/current_humans_action
*  - /agent_monitor/factList
*
* \section Published topics
*   - /action_executor/current_robot_action
*
* \section Services provided
*   - /action_executor/stop
*
* \section Parameters
* - action_executor/restPosition/(right/left): rest gtp positions for the robot arms
  - action_executor/shouldUseRightHand: flag to impose the use of the right arm
  - action_executor/nbPlanMaxGTP: nb max of time tryin to get a plan from gtp
  - action_executor/humanSafetyJoint: human joint to monitor for safety during action execution
  - action_executor/safetyThreshold: distance to human safety joint under which one the robot should stop
  - action_executor/gripperThreshold: value under which one the gripper is considered empty
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
* - action_lib
*
* Run dependencies
* - Toaster
*/

/** @file mainpage.h
* @brief Main page of the action executor doxygen documentation
*
*/
/** \mainpage Action Executor
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows the robot to execute high level actions.
* The actions currently implemented are:
*   - Pick
*   - Place
*   - Drop
*   - Move to
*   - Place reachable
*   - Scan
*   - Pick and drop
*   - Pick and place
*   - Pick and place reachable
*
* For action the node:
*   - checks its precondition and find the possible refinement for high level objects
*   - plans the necessary trajectories
*   - executes the action
*   - aplies/checks the post conditions
*
* \section Suscribed topics
*  - /human_monitor/current_humans_action: current action of the human (supervisor_msgs/ActionsList)
*  - /agent_monitor/factList: toaster facts concerning agents (toaster_msgs/FactList)
*
* \section Published topics
*   - /action_executor/current_robot_action: current action of the robot (supervisor_msgs/Action)
*   - /data_manager/add_data/previous_actions: previous actions of the robot (supervisor_msgs/ActionsList)
*
* \section Services provided
*   - /action_executor/stop: stop the current robot action (std_srvs/Empty)
*
* \section Parameters
* - action_executor/restPosition/(right,left): rest gtp positions for the robot arms (string)
  - action_executor/shouldUseRightHand: flag to impose the use of the right arm (bool)
  - action_executor/nbPlanMaxGTP: nb max of time tryin to get a plan from gtp (int)
  - action_executor/humanSafetyJoint: human joint to monitor for safety during action execution (string)
  - action_executor/safetyThreshold: distance to human safety joint under which one the robot should stop (double)
  - action_executor/gripperThreshold: value under which one the gripper is considered empty (double)
  - action_executor/humanCost: human considered to compute costs (string)
  - action_executor/uniqueSupports: support which can only have one object on it (list of string)
  - action_executor/noExec: allow to perform actions without the execution part, in simulation (bool)
  - action_executor/nolanning: allow to perform actions without the planning part, in simulation and if noExec is true (bool)
  - action_executor/planningSupport/<object>: replace the name of support by the name in parameter for planning, (string)
  - action_executor/points/<object>/<support>/(x,y,theta): allow to specify a specific point for planning a place (double)
*
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
* - action_lib
* - pr2motion
* - gtp_msgs
*
* Run dependencies
* - Toaster
* - gtp (if noPlanning = false)
* - pr2motion (if noExec = false)
*/

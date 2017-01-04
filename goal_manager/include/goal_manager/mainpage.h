/** @file mainpage.h
* @brief Main page of the goal manager doxygen documentation
*
*/
/** \mainpage Goal Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to manage the goals of the robot.
*
* It is possible to:
*   - Add a new goal to perform
*   - Cancel the current or a pending goal
*
* \section Published topics
*   - goal_manager/goalsState
*
* \section Services provided
*   - goal_manager/new_goal
*   - goal_manager/cancel_goal
*   - goal_manager/end_goal
*
* \section Parameters
* - goal_manager/goals/names: names of the possible goals
* - goal_manager/goals/<goal>_actors: possible actors for the goal <goal>
* - goal_manager/goals/<goal>_objective: objective for the goal <goal>
*
* \section Dependencies
* Build dependecies:
* - supervisor_msgs
*
*/

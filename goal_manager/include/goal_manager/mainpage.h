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
*   - goal_manager/goalsList: list of currents, past and pending goals (supervisor_msgs/GoalsList)
*
* \section Services provided
*   - goal_manager/new_goal: add a new goal to the pending list (supervisor_msgs/String)
*   - goal_manager/cancel_goal: cancel a goal (supervisor_msgs/String)
*   - goal_manager/end_goal: end the current goal (supervisor_msgs/String)
*
* \section Parameters
* - goal_manager/goals/names: names of the possible goals (list of strings)
* - goal_manager/goals/<goal>_actors: possible actors for the goal <goal> (list of strings)
* - goal_manager/goals/<goal>_objective: objective for the goal <goal> (list of string (facts as a string))
*
* \section Dependencies
* Build dependecies:
* - supervisor_msgs
*
*/

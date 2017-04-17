/** @file mainpage.h
* @brief Main page of the mental_states manager doxygen documentation
*
*/
/** \mainpage Mental State Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to constently estimate the mental states of human agents.
*
* \section Suscribed topics
*   - database_manager/tables: knowledge of agents from the database (toaster_msgs/DatabaseTables)
*   - goal_manager/goalsList: goals of the robot (supervisor_msgs/GoalsList)
*   - supervisor/previous_actions: previous actions of all agents (supervisor_msgs/ActionsList)
*   - plan_elaboration/plan: current plan of the robot (supervisor_msgs/SharedPlan)
*   - action_executor/current_robot_action: current action of the robot (supervisor_msgs/Action)
*   - dialogue_node/infoGiven: informations given to/by the robot (supervisor_msgs/InfoGiven)
*
* \section Published topics
*   - /mental_states/mental_states: mental states of all humans (supervisor_msgs/MentalStatesList)
*
* \section Parameters
*   - mental_states/nonObservableFacts: list of facts which are considered non observable (list of string)
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
*
* Run dependencies
* - Toaster
*/

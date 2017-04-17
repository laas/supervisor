/** @file mainpage.h
* @brief Main page of the plan elaboration doxygen documentation
*
*/
/** \mainpage Plan elaboration
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to find a plan to achieve a goal
*
* It is possible to:
*
* Look for a new plan
* Evaluate the current plan after an action alocation
* It is possible to add specific facts computations for a task between looking for a plan. The following tasks are available:
*  - Scan
*  - Blocks
*
* \section Published topics
*   - plan_elaboration/plan: The current shared plan
*
* \section Subscribed topics
*   - /goal_manager/goalsList: List of robot goals
*
* \section Services provided
*   - plan_elaboration/end_plan: Calls when a plan ends
*
* \section Parameters
* - plan_elaboration/Agents: name of the agents used for planning (list of string)
* - plan_elaboration/HATP_actions/<action>: translation of the HATP action names (string)
* - plan_elaboration/nbMaxTry: number of time to call HATP if no plan (int)
* - plan_elaboration/toIgnoreFactsForXAgent: the facts to ignore when computing the x agents facts (list of string)
* - plan_elaboration/domains/<domain>/facts: the facts use for planning (list of string)
* - plan_elaboration/domains/<domain>/method: the name of the HATP method used for planning (string)
* - plan_elaboration/domains/<domain>/param:s the parameters given to HATP for planning (list of string)
* - plan_elaboration/domains/SCAN/scanArea: the name of the are where to scan for the scan task (string)
* - plan_elaboration/domains/BLOCKS/stackArea: the name of the are where to stack for the blocks task (string)
*
* \section Dependencies
* Build dependecies:
* - supervisor_msgs
*
*/

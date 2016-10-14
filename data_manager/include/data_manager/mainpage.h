/** @file mainpage.h
* @brief Main page of the data manager doxygen documentation
*
*/
/** \mainpage Data Manager
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to concatenate all data usefull to supervision and publish them in appropriate topics.
* This concerns only data which can come from several modules. They are:
*   - The list of actions to do
*   - The list of previous actions
* For each data, the node can:
*   - concatenate data from several topics (topics name in param)
*   - add a data to a list (must be published on /data_manager/add_data/'data_name')
*   - remove a data to a list (must be published on /data_manager/rm_data/'data_name')
*
* \section Published topics
*   - supervisor/actions_todo
*   - supervisor/previous_actions
*
* \section Parameters
*  - actions_todo_topics: topics where to read actions to do
*  - previous_actions_topics: topics where to read previous actions
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
*
* Run dependencies
* - Toaster
*/

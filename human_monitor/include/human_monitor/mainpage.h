/** @file mainpage.h
* @brief Main page of the human monitor doxygen documentation
*
*/
/** \mainpage Human Monitor
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to recognize and simulate humans actions.
* The actions implemented are:
*   - Pick
*   - Place
*   - Drop
*
* \section Suscribed topics
*   - /agent_manager/factList
*
* \section Published topics
*   - /human_monitor/current_humans_action
*   - /data_manager/add_data/previous_actions
*
* \section Parameters
*  - human_monitor/shouldDetect: boolean which indicate if the actions detection should be activated
   - human_monitor/rightHand: the name of the toaster human right hand
   - human_monitor/threshold/pick: distance to objects for pick action
   - human_monitor/threshold/place: distance to supports for place action
   - human_monitor/threshold/drop: distance to containers for drop action
   - human_monitor/threshold/disengage: distance to objects for disengaging
   - human_monitor/replacementSupport/<support>: replacement support for the place detection
* Need launchs/Entities.yaml to run
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
*
* Run dependencies
* - Toaster
*/

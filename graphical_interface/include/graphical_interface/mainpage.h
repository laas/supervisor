/** @file mainpage.h
* @brief Main page of the graphical interface doxygen documentation
*
*/
/** \mainpage Graphical Interface
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node is a graphical interface for the supervision system
*
* \section Tabs
* The following tabs are available and can be activated/desactivated through parameters
*  - Database management
*  - Robot actions management
*  - Humans actions management
*
* \section Parameters
*  - graphical_interface/databaseTab: activate the database management tab
*  - graphical_interface/actionTab: activate the robot actions management tab
*  - graphical_interface/humanTab: activate the human actions management tab
*  - graphical_interface/possibleActions: possible actions the robot can execute
*  - graphical_interface/moveToPositions: possible moveTo positions for the robot
* Need launchs/Entities.yaml and launchs/General.yaml to run
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

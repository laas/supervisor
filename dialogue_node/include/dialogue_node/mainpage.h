/** @file mainpage.h
* @brief Main page of the dialogue node doxygen documentation
*
*/
/** \mainpage Dialogue Node
*
* @authors Sandra Devin (sdevin@laas.fr)
*
* Status: maintained, documented
*
* This node allows to to simulate dialogue (in a very simple way)
*
* \section Suscribed topics
*   - /graphical_interface/boolAnswer: answer to bool question from graphical interface (std_msgs/Bool)
*
* \section Published topics
*   - /dialogue_node/info_given: the information given by/to the robot (supervisor_msgs/Info)
*   - /dialogue_node/isSpeaking: true when the robot is speaking (std_msgs/Bool)
*
* \section Services
*   - dialogue_node/say: verbalize a sentence (supervisor_msgs/String)
*   - dialogue_node/ask: ask a question (supervisor_msgs/Ask)
*   - dialogue_node/give_info: give an information (supervisor_msgs/GiveInfo)
*
* \section Parameters
*  - dialogue_node/shouldSpeak: if the robot should really verbalize the sentences (bool)
*  - dialogue_node/acapelaVoice: name of the voice to use for acapela (string)
*  - dialogue_node/timeWaitAnswer: time to wait to an answer to a question (double)
*  - dialogue_node/entitiesTranslation/<entity>: name to verbalize an entity (string)
*  - dialogue_node/properties/translationTrue/<property>: way to say a property when it is true (string)
*  - dialogue_node/properties/translationFalse/<property>: way to say a property when it is false (string)
*  - dialogue_node/properties/translationAsk/<property>: way to say a property when we ask a question (string)
*  - dialogue_node/properties/isReverse/<property>: true if we need to reverse the sentence with this property (bool)s
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
* - acapela (can be removed by commenting line in CMakeList.txt)
*
*/

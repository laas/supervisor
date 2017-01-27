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
*   - /graphical_interface/boolAnswer
*
* \section Services
*   - dialogue_node/say
*   - dialogue_node/ask
*   - dialogue_node/give_info
*
* \section Parameters
*  - dialogue_node/shouldSpeak: if the robo should really verbalize the sentences
*  - dialogue_node/acapelaVoice: name of the voice to use for acapela
*  - dialogue_node/timeWaitAnswer: time to wait to an answer to a question
*  - dialogue_node/entitiesTranslation/<entity>: name to verbalize an entity
*  - dialogue_node/properties/translationTrue/<property>: way to say a property when it is true
*  - dialogue_node/properties/translationFalse/<property>: way to say a property when it is false
*  - dialogue_node/properties/translationAsk/<property>: way to say a property when we ask a question
*  - dialogue_node/properties/isReverse/<property>: true if we need to reverse the sentence with this property
*
* \section Dependencies
* Build dependecies:
* - toaster_msgs
* - supervisor_msgs
* - acapela (can be removed by commenting line in CMakeList.txt)
*
*/

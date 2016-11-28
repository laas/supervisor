/**
author Sandra Devin
Template to create a new action

Todo:
    -replace Template by the new action name
    -fill the following function with whatever is needed
**/

#include "action_executor/Actions/template.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
Template::Template(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    //here we fill the actions parameters (e.g. object) with the action definition
}

/**
 * \brief Precondition of the template action:
 *    - describe here the preconditions
 * @return true if the preconditions are checked
 * */
bool Template::preconditions(){

    //here we check action preconditions

    std::vector<toaster_msgs::Fact> precsTocheck;
    //we can direclty check for facts presence in the robot table of the database

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the template action:
 *    - describe here the planning process
 * @return true if the planning succeed
 * */
bool Template::plan(){

   //Here we fill the needed information for gtp (fill free to use other planner if needed)
   std::vector<gtp_ros_msgs::Role> agents;
   std::vector<gtp_ros_msgs::Role> objects;
   std::vector<gtp_ros_msgs::Point> points;
   std::vector<gtp_ros_msgs::MiscData> datas;

   //do not forget to replace action_name in the following line
   std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP("action_name", agents, objects, datas, points);
   gtpActionId_ = answer.first;

   if(gtpActionId_ == -1){
       return false;
    }else{
       subSolutions_ = answer.second;
       return true;
    }

    return true;
}

/**
 * \brief Execution of the template action:
 *    - describe here the execution process
 * @return true if the execution succeed
 * */
bool Template::exec(Server* action_server){

   //Here we execute the action
   //This line simply execute the solution given by gtp
   return execAction(gtpActionId_, subSolutions_, true, action_server);

}

/**
 * \brief Post conditions of the template action:
 *    - describe here the post conditions
 * @return true if the post-conditions are ok
 * */
bool Template::post(){

    //Here we check/apply post conditions

    return true;
}

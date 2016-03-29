/**
 **/
#include "../include/graphical_interface/main_window.hpp"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      actionClient_("supervisor/action_executor", true)
{
    ui.setupUi(this);
    setWindowTitle("PR2 supervisor");
    node_.getParam("/robot/name", robotName_);

    //we retrieve the possible actions from param of the .yaml file
    vector<string> actionNames;
    node_.getParam("/highLevelActions/names", actionNames);
    for(vector<string>::iterator it = actionNames.begin(); it != actionNames.end(); it++){
        ui.comboBoxActionName->addItem(it->c_str());
        ui.comboBoxHumanActionName->addItem(it->c_str());
    }

    //we retrieve the possible agents from param of the .yaml file
    vector<string> agents;
    node_.getParam("/entities/agents", agents);
    for(vector<string>::iterator it = agents.begin(); it != agents.end(); it++){
        if(*it != robotName_){
            ui.comboBoxHumanAgent->addItem(it->c_str());
        }
        ui.comboBoxAgentKnowName->addItem(it->c_str());
    }

    //we retrieve the possible objects from param of the .yaml file
    vector<string> objects;
    node_.getParam("/entities/objects", objects);
    for(vector<string>::iterator it = objects.begin(); it != objects.end(); it++){
        ui.comboBoxActionObject->addItem(it->c_str());
        ui.comboBoxHumanActionObject->addItem(it->c_str());
    }


    //we retrieve the possible supports from param of the .yaml file
    vector<string> supports;
    node_.getParam("/entities/supports", supports);
    for(vector<string>::iterator it = supports.begin(); it != supports.end(); it++){
        ui.comboBoxActionSupport->addItem(it->c_str());
        ui.comboBoxHumanActionSupport->addItem(it->c_str());
    }


    //we retrieve the possible containers from param of the .yaml file
    vector<string> containers;
    node_.getParam("/entities/containers", containers);
    for(vector<string>::iterator it = containers.begin(); it != containers.end(); it++){
        ui.comboBoxActionContainer->addItem(it->c_str());
        ui.comboBoxHumanActionContainer->addItem(it->c_str());
    }


    //we retrieve the possible positions from param of the .yaml file
    vector<string> positions;
    node_.getParam("/moveToPositions/names", positions);
    for(vector<string>::iterator it = positions.begin(); it != positions.end(); it++){
        ui.comboBoxActionPosition->addItem(it->c_str());
    }


    //we retrieve the possible goals from param of the .yaml file
    vector<string> goals;
    node_.getParam("/goals/names", goals);
    for(vector<string>::iterator it = goals.begin(); it != goals.end(); it++){
        ui.comboBoxGoalName->addItem(it->c_str());
    }


    actionClient_.waitForServer();


}

MainWindow::~MainWindow() {}


/*************************************
 * Action tab
 *************************************/

/*
Send an action to the action executor (simple non aware execution without head control)
*/
void MainWindow::on_pushButtonExecuteAction_clicked()
{
    //Getting parameters
    string actionName = ui.comboBoxActionName->currentText().toStdString();
    string actionObject = ui.comboBoxActionObject->currentText().toStdString();
    string actionSupport = ui.comboBoxActionSupport->currentText().toStdString();
    string actionContainer = ui.comboBoxActionContainer->currentText().toStdString();
    string position = ui.comboBoxActionPosition->currentText().toStdString();

    //creating the action with the good parameters coming from higl level actions
    supervisor_msgs::ActionExecutorGoal goal;
    goal.action.name = actionName;
    goal.action.id = -1;
    goal.action.actors.push_back(robotName_);
    string paramTopic = "highLevelActions/";
    paramTopic = paramTopic + actionName + "_param";
    vector<string> params;
    node_.getParam(paramTopic, params);
    for(vector<string>::iterator it = params.begin(); it != params.end(); it++){
        if(*it == "mainObject"){
             goal.action.parameters.push_back(actionObject);
        }else if(*it == "supportObject"){
            goal.action.parameters.push_back(actionSupport);
       }else if(*it == "containerObject"){
            goal.action.parameters.push_back(actionContainer);
       }else if(*it == "position"){
            goal.action.parameters.push_back(position);
       }
    }

    //Sending the actions
    actionClient_.sendGoal(goal);
}

/*
Ask an action: send the action to the mental state manager (complete execution)
*/
void MainWindow::on_pushButtonAskAction_clicked()
{
    //Getting parameters
    string actionName = ui.comboBoxActionName->currentText().toStdString();
    string actionObject = ui.comboBoxActionObject->currentText().toStdString();
    string actionSupport = ui.comboBoxActionSupport->currentText().toStdString();
    string actionContainer = ui.comboBoxActionContainer->currentText().toStdString();
    string position = ui.comboBoxActionPosition->currentText().toStdString();

    //creating the action
    ros::ServiceClient action_state = node_.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
    supervisor_msgs::ChangeState srv_astate;
    srv_astate.request.type = "action";
    srv_astate.request.action.name = actionName;
    srv_astate.request.action.id = -1;
    srv_astate.request.action.actors.push_back(robotName_);
    string paramTopic = "highLevelActions/";
    paramTopic = paramTopic + actionName + "_param";
    vector<string> params;
    node_.getParam(paramTopic, params);
    for(vector<string>::iterator it = params.begin(); it != params.end(); it++){
        if(*it == "mainObject"){
             srv_astate.request.action.parameters.push_back(actionObject);
        }else if(*it == "supportObject"){
            srv_astate.request.action.parameters.push_back(actionSupport);
       }else if(*it == "containerObject"){
            srv_astate.request.action.parameters.push_back(actionContainer);
       }else if(*it == "position"){
            srv_astate.request.action.parameters.push_back(position);
       }
    }

    srv_astate.request.state = "ASKED";

    //Sending the action
    if (!action_state.call(srv_astate)) {
       ROS_ERROR("Failed to call service mental_state/change_state");
       return;
    }
}

/*
Stop the action execution
*/
void MainWindow::on_pushButtonStopAction_clicked()
{
    actionClient_.cancelGoal();
}



/*************************************
 * Human Simu tab
 *************************************/



void MainWindow::on_pushButtonSendHumanAction_clicked()
{
    //Getting parameters
    string agent = ui.comboBoxHumanAgent->currentText().toStdString();
    string actionName = ui.comboBoxHumanActionName->currentText().toStdString();
    string actionObject = ui.comboBoxHumanActionObject->currentText().toStdString();
    string actionSupport = ui.comboBoxHumanActionSupport->currentText().toStdString();
    string actionContainer = ui.comboBoxHumanActionContainer->currentText().toStdString();

    //creating the action
    ros::ServiceClient human_action = node_.serviceClient<supervisor_msgs::HumanActionSimu>("human_monitor/human_action_simu");
    supervisor_msgs::HumanActionSimu srv_haction;
    srv_haction.request.actionName = actionName;
    srv_haction.request.agent = agent;
    srv_haction.request.object = actionObject;
    srv_haction.request.support = actionSupport;
    srv_haction.request.container = actionContainer;

    //Sending the action
    if (!human_action.call(srv_haction)) {
       ROS_ERROR("Failed to call service human_monitor/human_action_simu");
       return;
    }
}


/*************************************
 * Goal tab
 *************************************/


void MainWindow::on_pushButtonExecuteGoal_clicked()
{
    //Getting parameters
    string goal = ui.comboBoxGoalName->currentText().toStdString();

    //sending the goal to the goal manager
    ros::ServiceClient new_goal = node_.serviceClient<supervisor_msgs::NewGoal>("goal_manager/new_goal");
    supervisor_msgs::NewGoal srv;
    srv.request.goal = goal;
    if (!new_goal.call(srv)) {
       ROS_ERROR("Failed to call service goal_manager/new_goal");
       return;
    }

}



/*************************************
 * Knowledge tab
 *************************************/


void MainWindow::on_pushButtonPrintKnowledge_clicked()
{
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");

    //get the agent name
    string agent = ui.comboBoxAgentKnowName->currentText().toStdString();

    //get all the facts for the agent
    supervisor_msgs::GetInfo srv;
    srv.request.info = "ALL_FACTS";
    srv.request.agent = agent;
    if (client.call(srv)) {
        //We sort the facts by type
        vector<toaster_msgs::Fact> envFacts, actionFacts, planFacts, goalFacts;
        for(vector<toaster_msgs::Fact>::iterator it = srv.response.facts.begin(); it != srv.response.facts.end(); it++){
            if(it->property == "actionState"){
                 actionFacts.push_back(*it);
            }else if(it->property == "planState"){
                planFacts.push_back(*it);
           }else if(it->property == "goalState"){
                goalFacts.push_back(*it);
           }else{
                envFacts.push_back(*it);
           }
        }
        //and we print all the facts
        string toPrint = "---Environment---\n";
        for(vector<toaster_msgs::Fact>::iterator it = envFacts.begin(); it != envFacts.end(); it++){
            toPrint = toPrint + it->subjectId + " " + it->property + " " + it->targetId + "\n";
        }
        toPrint = toPrint + "---Goals---\n";
        for(vector<toaster_msgs::Fact>::iterator it = goalFacts.begin(); it != goalFacts.end(); it++){
            toPrint = toPrint + it->subjectId + " " + it->property + " " + it->targetId + "\n";
        }
        toPrint = toPrint + "---Plans---\n";
        for(vector<toaster_msgs::Fact>::iterator it = planFacts.begin(); it != planFacts.end(); it++){
            toPrint = toPrint + it->subjectId + " " + it->property + " " + it->targetId + "\n";
        }
        toPrint = toPrint + "---Actions---\n";
        for(vector<toaster_msgs::Fact>::iterator it = actionFacts.begin(); it != actionFacts.end(); it++){
            toPrint = toPrint + it->subjectId + " " + it->property + " " + it->targetId + "\n";
        }
        ui.textBrowserKnowledge->setText(QString::fromStdString(toPrint));

    }else{
       ROS_ERROR("Failed to call service mental_state/get_info");
    }

}
void MainWindow::on_pushButtonSeeActions_clicked()
{
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    supervisor_msgs::GetInfo srv;
    srv.request.info = "ACTIONS";
    ui.textBrowserActions->setText("");
    if (client.call(srv)) {
        string toPrint;
        for(vector<supervisor_msgs::ActionMS>::iterator it = srv.response.actions.begin(); it != srv.response.actions.end(); it++){
            toPrint = toPrint + boost::lexical_cast<string>(it->id) + "\n" + it->name + "\n" + "Actors: ";
            for(vector<string>::iterator itt = it->actors.begin(); itt != it->actors.end(); itt++){
                toPrint = toPrint + *itt + " ";
            }
            toPrint = toPrint + "\n" + "Parameters: ";
            for(vector<string>::iterator itt = it->parameters.begin(); itt != it->parameters.end(); itt++){
                toPrint = toPrint + *itt + " ";
            }
            toPrint = toPrint + "\n" + "-----------------\n";
        }
        ui.textBrowserActions->setText(QString::fromStdString(toPrint));
    }else{
        ROS_ERROR("Failed to call service mental_state/get_info");
   }
}

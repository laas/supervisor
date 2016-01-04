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

    //Add possible actions name
    ui.comboBoxActionName->addItem("pick");
    ui.comboBoxActionName->addItem("place");
    ui.comboBoxActionName->addItem("pickandplace");

    //Add possible actions object
    ui.comboBoxActionObject->addItem("RED_CUBE");

    //Add possible actions support
    ui.comboBoxActionSupport->addItem("TABLE_4");

    //Add possible actions agent
    ui.comboBoxActionAgent->addItem("HERAKLES_HUMAN1");
    ui.comboBoxActionAgent->addItem("HERAKLES_HUMAN1");

    actionClient_.waitForServer();


}

MainWindow::~MainWindow() {}


/*
Send an action to the action executor (simple non aware execution without head control)
*/
void MainWindow::on_pushButtonExecuteAction_clicked()
{
    //Getting parameters
    string actionName = ui.comboBoxActionName->currentText().toStdString();
    string actionObject = ui.comboBoxActionObject->currentText().toStdString();
    string actionSupport = ui.comboBoxActionSupport->currentText().toStdString();
    string actionAgent = ui.comboBoxActionAgent->currentText().toStdString();

    //creating the action
    supervisor_msgs::ActionExecutorGoal goal;
    goal.action.name = actionName;
    goal.action.id = -1;
    goal.action.actors.push_back(robotName_);
    if(actionName == "pick"){
          goal.action.parameters.push_back(actionObject);
    }else if(actionName == "place"){
        goal.action.parameters.push_back(actionObject);
        goal.action.parameters.push_back(actionSupport);
    }else if(actionName == "pickandplace"){
        goal.action.parameters.push_back(actionSupport);
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
    string actionAgent = ui.comboBoxActionAgent->currentText().toStdString();

    //creating the action
    ros::ServiceClient action_state = node_.serviceClient<supervisor_msgs::ActionState>("mental_state/action_state");
    supervisor_msgs::ActionState srv_astate;
    srv_astate.request.action.name = actionName;
    srv_astate.request.action.id = -1;
    srv_astate.request.action.actors.push_back(robotName_);
    if(actionName == "pick"){
          srv_astate.request.action.parameters.push_back(actionObject);
    }else if(actionName == "place"){
        srv_astate.request.action.parameters.push_back(actionObject);
        srv_astate.request.action.parameters.push_back(actionSupport);
    }else if(actionName == "pickandplace"){
        srv_astate.request.action.parameters.push_back(actionSupport);
    }

    srv_astate.request.state = "ASKED";

    //Sending the action
    if (!action_state.call(srv_astate)) {
       ROS_ERROR("Failed to call service mental_state/action_state");
       return;
    }
}

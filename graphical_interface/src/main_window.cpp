/**
 **/
#include "../include/graphical_interface/main_window.hpp"

/**
 * \brief Construction of the class
 * */
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      actionClient_("supervisor/action_executor", true)
{
    ui.setupUi(this);
    setWindowTitle("PR2 supervisor");
    node_.getParam("/supervisor/robot/name", robotName_);
    node_.getParam("/supervisor/simu", simu_);
    node_.getParam("/supervisor/waitActionServer", waitActionServer_);
    node_.getParam("/graphical_interface/goalTab", goalTab_);
    node_.getParam("/graphical_interface/databaseTab", databaseTab_);
    node_.getParam("/graphical_interface/actionTab", actionTab_);
    node_.getParam("/graphical_interface/humanTab", dialogueTab_);
    node_.getParam("/graphical_interface/dialogueTab", humanTab_);

    //we retrieve the possible agents from param of the .yaml file
    std::vector<std::string> agents;
    node_.getParam("/entities/agents", agents);
    for(std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); it++){
        if(*it != robotName_){
            ui.comboBoxAgentAction->addItem(it->c_str());
            ui.comboBoxHuman->addItem(it->c_str());
            ui.comboBoxHumanDia->addItem(it->c_str());
        }
        ui.comboBoxAgentDB->addItem(it->c_str());
    }

    //we retrieve the possible objects from param of the .yaml file
    std::vector<std::string> objects;
    node_.getParam("/entities/objects", objects);
    for(std::vector<std::string>::iterator it = objects.begin(); it != objects.end(); it++){
        ui.comboBoxObjectAction->addItem(it->c_str());
        ui.comboBoxObjectDetachAction->addItem(it->c_str());
        ui.comboBoxObjectHuman->addItem(it->c_str());
        ui.comboBoxObjectDetachHuman->addItem(it->c_str());
    }

    //we retrieve the possible supports from param of the .yaml file
    std::vector<std::string> supports;
    node_.getParam("/entities/supports", supports);
    for(std::vector<std::string>::iterator it = supports.begin(); it != supports.end(); it++){
        ui.comboBoxSupportAction->addItem(it->c_str());
        ui.comboBoxSupportHuman->addItem(it->c_str());
    }

    //we retrieve the possible containers from param of the .yaml file
    std::vector<std::string> containers;
    node_.getParam("/entities/containers", containers);
    for(std::vector<std::string>::iterator it = containers.begin(); it != containers.end(); it++){
        ui.comboBoxContainerAction->addItem(it->c_str());
        ui.comboBoxContainerHuman->addItem(it->c_str());
    }

    //we retrieve the possible actions from param of the .yaml file
    std::vector<std::string> possibleActions;
    node_.getParam("/graphical_interface/possibleActions", possibleActions);
    for(std::vector<std::string>::iterator it = possibleActions.begin(); it != possibleActions.end(); it++){
        ui.comboBoxNameAction->addItem(it->c_str());
    }

    //we retrieve the possible positions from param of the .yaml file
    std::vector<std::string> positions;
    node_.getParam("/graphical_interface/moveToPositions", positions);
    for(std::vector<std::string>::iterator it = positions.begin(); it != positions.end(); it++){
        ui.comboBoxPositionAction->addItem(it->c_str());
    }

    //we retrieve the possible goals from param of the .yaml file
    std::vector<std::string> goals;
    node_.getParam("/goal_manager/goals/names", goals);
    for(std::vector<std::string>::iterator it = goals.begin(); it != goals.end(); it++){
        ui.comboBoxGoals->addItem(it->c_str());
    }



    //desactivate tabs if needed
    if(!goalTab_){
        ui.Interface->setTabEnabled(0, false);
    }

    if(!databaseTab_){
        ui.Interface->setTabEnabled(1, false);
    }

    if(!actionTab_){
        ui.Interface->setTabEnabled(2, false);
        ROS_INFO("[graphical_interface] Waiting action client");
        actionClient_.waitForServer();
    }

    if(!humanTab_){
        ui.Interface->setTabEnabled(3, false);
    }

    if(!dialogueTab_){
        ui.Interface->setTabEnabled(4, false);
    }

    client_set_db_ = node_.serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");
    client_get_db_ = node_.serviceClient<toaster_msgs::GetInfoDB>("database_manager/get_info");
    client_execute_db_ = node_.serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    client_stop_action_ = node_.serviceClient<std_srvs::Empty>("action_executor/stop");
    client_detach_object_ = node_.serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    client_human_action_ = node_.serviceClient<toaster_msgs::RemoveFromHand>("human_monitor/human_action_simu");
    client_send_goal_ = node_.serviceClient<supervisor_msgs::String>("goal_manager/new_goal");
    client_cancel_goal_ = node_.serviceClient<supervisor_msgs::String>("goal_manager/cancel_goal");
    client_say_ = node_.serviceClient<supervisor_msgs::String>("dialogue_node/say");
    client_give_info_ = node_.serviceClient<supervisor_msgs::GiveInfo>("dialogue_node/give_info");


    boolPub_ = node_.advertise<std_msgs::Bool>("graphical_interface/boolAnswer", 1);

}

MainWindow::~MainWindow() {}

/** ***************************************
 * Database Tab
 * ****************************************/

/**
 * \brief Push button to add a fact
 * */
void MainWindow::on_pushButtonAddDB_clicked()
{
    //we get the fact content
    std::string subject = ui.textSubject->toPlainText().toStdString();
    if(subject == ""){
        ROS_WARN("[graphical_interface] No subject specified: NULL instead");
        subject = "NULL";
    }
    std::string target = ui.textTarget->toPlainText().toStdString();
    if(target == ""){
        ROS_WARN("[graphical_interface] No target specified: NULL instead");
        target = "NULL";
    }
    std::string property = ui.textProperty->toPlainText().toStdString();
    if(property == ""){
        ROS_ERROR("[graphical_interface] No property specified: abort");
        return;
    }

    //we add it to the db
    ROS_INFO("[graphical_interface] adding the fact: %s %s %s", subject.c_str(), property.c_str(), target.c_str());
    std::vector<toaster_msgs::Fact> facts;
    toaster_msgs::Fact fact;
    fact.subjectId = subject;
    fact.property = property;
    fact.targetId = target;
    facts.push_back(fact);
    toaster_msgs::SetInfoDB srv;
    srv.request.agentId = robotName_;
    srv.request.facts = facts;
    srv.request.infoType = "FACT";
    srv.request.add = true;
    if (!client_set_db_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service database_manager/set_info");
    }
}

/**
 * \brief Push button to remove a fact
 * */
void MainWindow::on_pushButtonRmDB_clicked()
{
    //we get the fact content
    std::string subject = ui.textSubject->toPlainText().toStdString();
    if(subject == ""){
        ROS_WARN("[graphical_interface] No subject specified: NULL instead");
        subject = "NULL";
    }
    std::string target = ui.textTarget->toPlainText().toStdString();
    if(target == ""){
        ROS_WARN("[graphical_interface] No target specified: NULL instead");
        target = "NULL";
    }
    std::string property = ui.textProperty->toPlainText().toStdString();
    if(property == ""){
        ROS_ERROR("[graphical_interface] No property specified: abort");
        return;
    }

    //we add it to the db
    ROS_INFO("[graphical_interface] removing the fact: %s %s %s", subject.c_str(), property.c_str(), target.c_str());
    std::vector<toaster_msgs::Fact> facts;
    toaster_msgs::Fact fact;
    fact.subjectId = subject;
    fact.property = property;
    fact.targetId = target;
    facts.push_back(fact);
    toaster_msgs::SetInfoDB srv;
    srv.request.agentId = robotName_;
    srv.request.facts = facts;
    srv.request.infoType = "FACT";
    srv.request.add = false;
    if (!client_set_db_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service database_manager/set_info");
    }
}

/**
 * \brief Push button to print a fact
 * */
void MainWindow::on_pushButtonPrintDB_clicked()
{
    //create the request
    std::string request = "SELECT * from fact_table_" + robotName_ + " where";
    bool find = false;
    std::string subject = ui.textSubject->toPlainText().toStdString();
    if(subject != ""){
        request = request + " subject_id = " + subject;
        find = true;
    }
    std::string target = ui.textTarget->toPlainText().toStdString();
    if(target != ""){
        if(find){
            request = request + " and";
            find = true;
        }
        request = request + " target_id = " + target;
        find = true;
    }
    std::string property = ui.textProperty->toPlainText().toStdString();
    if(property != ""){
        if(find){
            request = request + " and";
        }
        request = request + " predicate = " + property;
        find = true;
    }
    if(!find){
        ROS_ERROR("[graphical_interface] You should precise at least one part of the fact");
        return;
    }

    //we ask the corresponding facts
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "SQL";
    srv.request.order = request;
    if (client_execute_db_.call(srv)){
        //we print them
        if(srv.response.results.size()%3 != 0){
            ROS_ERROR("[graphical_interface] Wrong database answer: not a set of 3 to produce facts!");
            return;
        }
        std::string toPrint;
        for(int i = 0; i < srv.response.results.size(); i = i+3){
            subject = srv.response.results[i];
            property = srv.response.results[i+1];
            target = srv.response.results[i+2];
            toPrint = toPrint + subject.c_str() + " " + property.c_str() + " " + target.c_str() + "\n";
        }
        ui.textDB->setText(QString::fromStdString(toPrint));
    }else{
        ROS_ERROR("[graphical_interface] Failed to call service database_manager/execute");
    }


}

/**
 * \brief Push button to print all facts in an agent table
 * */
void MainWindow::on_pushButtonPrintAllDB_clicked()
{
    //we ask all facts
    toaster_msgs::GetInfoDB srv;
    srv.request.type = "FACT";
    srv.request.subType = "ALL";
    srv.request.agentId = ui.comboBoxAgentDB->currentText().toStdString();
    if (client_get_db_.call(srv)){
        std::string toPrint;
        for(std::vector<toaster_msgs::Fact>::iterator it = srv.response.resFactList.factList.begin(); it != srv.response.resFactList.factList.end(); it++){
            toPrint = toPrint + it->subjectId.c_str() + " " + it->property.c_str() + " " + it->targetId.c_str() + "\n";
        }
        ui.textDB->setText(QString::fromStdString(toPrint));
    }else{
        ROS_ERROR("[graphical_interface] Failed to call service database_manager/get_info");
    }

}

/**
 * \brief Push button to print all facts in an agent table
 * */
void MainWindow::on_pushButtonResetDB_clicked()
{
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "EMPTY";
    srv.request.type = "ALL";
    if (!client_execute_db_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service database_manager/execute");
    }

}


/** ***************************************
 * Action Tab
 * ****************************************/

/**
 * \brief Push button to execute an action
 * */
void MainWindow::on_pushButtonExecuteAction_clicked()
{
    //we get all the parameters and construct the action
    supervisor_msgs::Action action;
    action.actors.push_back(robotName_);
    action.name = ui.comboBoxNameAction->currentText().toStdString();
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(ui.comboBoxObjectAction->currentText().toStdString());
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back(ui.comboBoxSupportAction->currentText().toStdString());
    action.parameter_keys.push_back("container");
    action.parameter_values.push_back(ui.comboBoxContainerAction->currentText().toStdString());
    action.parameter_keys.push_back("targetAgent");
    action.parameter_values.push_back(ui.comboBoxAgentAction->currentText().toStdString());
    action.parameter_keys.push_back("confName");
    action.parameter_values.push_back(ui.comboBoxPositionAction->currentText().toStdString());

    //we send it to the action executor
    supervisor_msgs::ActionExecutorGoal goal;
    goal.action = action;
    actionClient_.sendGoal(goal);


}

/**
 * \brief Push button to stop an action
 * */
void MainWindow::on_pushButtonStopAction_clicked()
{

    std_srvs::Empty srv;
    if (!client_stop_action_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service action_executor/stop");
    }
}

/**
 * \brief Push button to detach an object from the robot hand
 * */
void MainWindow::on_pushButtonDetach_clicked()
{

    toaster_msgs::RemoveFromHand srv;
    srv.request.objectId = ui.comboBoxObjectDetachAction->currentText().toStdString();
    if (!client_detach_object_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service pdg/remove_from_hand");
    }

}

/** ***************************************
 * Human action Tab
 * ****************************************/

/**
 * \brief Push button to execute a human action
 * */
void MainWindow::on_pushButtonExecuteHuman_clicked()
{

    //we get all the parameters and construct the action
    supervisor_msgs::Action action;
    action.actors.push_back(ui.comboBoxHuman->currentText().toStdString());
    action.name = ui.comboBoxNameActionHuman->currentText().toStdString();
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(ui.comboBoxObjectHuman->currentText().toStdString());
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back(ui.comboBoxSupportHuman->currentText().toStdString());
    action.parameter_keys.push_back("container");
    action.parameter_values.push_back(ui.comboBoxContainerHuman->currentText().toStdString());

    //we send it to the action executor
    supervisor_msgs::HumanAction srv;
    srv.request.agent = ui.comboBoxHuman->currentText().toStdString();
    srv.request.action = action;
    if (!client_human_action_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

}

/**
 * \brief Push button to detach an object from a human hand
 * */
void MainWindow::on_pushButtonDetachHuman_clicked()
{

    toaster_msgs::RemoveFromHand srv;
    srv.request.objectId = ui.comboBoxObjectDetachHuman->currentText().toStdString();
    if (!client_detach_object_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service pdg/remove_from_hand");
    }

}

/** ***************************************
 * Goal Tab
 * ****************************************/

/**
 * \brief Push button to send a new goal
 * */
void MainWindow::on_pushButtonSendGoal_clicked(){

    //we goal the corresponding service
    supervisor_msgs::String srv;
    srv.request.data = ui.comboBoxGoals->currentText().toStdString();
    if (!client_send_goal_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service goal_manager/new_goal");
    }
}

/**
 * \brief Push button to cancel a goal
 * */
void MainWindow::on_pushButtonCancelGoal_clicked()
{
    //we goal the corresponding service
    supervisor_msgs::String srv;
    srv.request.data = ui.comboBoxGoals->currentText().toStdString();
    if (!client_cancel_goal_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service goal_manager/cancel_goal");
    }
}

/** ***************************************
 * Dialogue Tab
 * ****************************************/

/**
 * \brief Push button to say a sentence
 * */
void MainWindow::on_pushButtonSay_clicked()
{

    //we get the sentence
    std::string sentence = ui.textDia->toPlainText().toStdString();

    //we send it to the dialogue module
    supervisor_msgs::String srv;
    srv.request.data = sentence;
    if (!client_say_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service dialogue_node/say");
    }
}

/**
 * \brief Push button to give a fact to the robot (simulated dialogue)
 * */
void MainWindow::on_pushButtonHumanSay_clicked()
{
    //we get the fact content
    std::string subject = ui.textSubjectDia->toPlainText().toStdString();
    if(subject == ""){
        ROS_WARN("[graphical_interface] No subject specified: NULL instead");
        subject = "NULL";
    }
    std::string target = ui.textTargetDia->toPlainText().toStdString();
    if(target == ""){
        ROS_WARN("[graphical_interface] No target specified: NULL instead");
        target = "NULL";
    }
    std::string property = ui.textPropertyDia->toPlainText().toStdString();
    if(property == ""){
        ROS_ERROR("[graphical_interface] No property specified: abort");
        return;
    }
    toaster_msgs::Fact fact;
    fact.subjectId = subject;
    fact.property = property;
    fact.targetId = target;

    //we send it to the dialogue module
    supervisor_msgs::GiveInfo srv;
    srv.request.toRobot = true;
    srv.request.type = "FACT";
    srv.request.partner = ui.comboBoxHumanDia->currentText().toStdString();
    srv.request.fact = fact;
    if (!client_give_info_.call(srv)){
        ROS_ERROR("[graphical_interface] Failed to call service dialogue_node/say");
    }
}

/**
 * \brief Push button to say yes
 * */
void MainWindow::on_pushButtonYes_clicked()
{

    std_msgs::Bool msg;
    msg.data = true;
    boolPub_.publish(msg);
}

/**
 * \brief Push button to say no
 * */
void MainWindow::on_pushButtonNo_clicked()
{

    std_msgs::Bool msg;
    msg.data = false;
    boolPub_.publish(msg);

}

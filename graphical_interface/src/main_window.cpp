/**
 **/
#include "../include/graphical_interface/main_window.hpp"

vector<string> toIgnoreFacts;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      actionClientSup_("supervisor/action_executor", true),
      actionClientTorso_("pr2motion/Torso_Move", true),
      actionClientRightArm_("pr2motion/Arm_Right_MoveToQGoal",true),
      actionClientLeftArm_("pr2motion/Arm_Left_MoveToQGoal",true),
      actionClientRightGripper_("pr2motion/Gripper_Right_Operate",true),
      actionClientLeftGripper_("pr2motion/Gripper_Left_Operate",true),
      actionClientHead_("pr2motion/Head_Move", true),
      actionClientGetQ_("pr2motion/GetQ", true)
{
    ui.setupUi(this);
    setWindowTitle("PR2 supervisor");
    node_.getParam("/robot/name", robotName_);

    boolAnswerPub_ = node_.advertise<supervisor_msgs::Bool>("graphical_interface/boolAnswer", 1000);

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


    //we retrieve the possible positions from param of the .yaml file
    vector<string> goals;
    node_.getParam("/goals/names", goals);
    for(vector<string>::iterator it = goals.begin(); it != goals.end(); it++){
        ui.comboBoxGoalName->addItem(it->c_str());
    }


    //we retrieve the possible goals from param of the .yaml file
    node_.getParam("/toIgnorePrint", toIgnoreFacts);
    node_.getParam("/waitActionServer", waitActionServer_);
    node_.getParam("/simu", simu_);

    //configure the values for robot control
    ui.TorsoValue->setRange(0.0, 0.3);
    ui.TorsoValue->setSingleStep(0.1);
    ui.Q1Right->setRange(-1.0, 1.0);
    ui.Q1Right->setSingleStep(0.1);
    ui.Q2Right->setRange(-1.0, 1.0);
    ui.Q2Right->setSingleStep(0.1);
    ui.Q3Right->setRange(-1.0, 1.0);
    ui.Q3Right->setSingleStep(0.1);
    ui.Q4Right->setRange(-1.0, 1.0);
    ui.Q4Right->setSingleStep(0.1);
    ui.Q5Right->setRange(-1.0, 1.0);
    ui.Q5Right->setSingleStep(0.1);
    ui.Q6Right->setRange(-1.0, 1.0);
    ui.Q6Right->setSingleStep(0.1);
    ui.Q7Right->setRange(-1.0, 1.0);
    ui.Q7Right->setSingleStep(0.1);
    ui.Q1Left->setRange(-1.0, 1.0);
    ui.Q1Left->setSingleStep(0.1);
    ui.Q2Left->setRange(-1.0, 1.0);
    ui.Q2Left->setSingleStep(0.1);
    ui.Q3Left->setRange(-1.0, 1.0);
    ui.Q3Left->setSingleStep(0.1);
    ui.Q4Left->setRange(-1.0, 1.0);
    ui.Q4Left->setSingleStep(0.1);
    ui.Q5Left->setRange(-1.0, 1.0);
    ui.Q5Left->setSingleStep(0.1);
    ui.Q6Left->setRange(-1.0, 1.0);
    ui.Q6Left->setSingleStep(0.1);
    ui.Q7Left->setRange(-1.0, 1.0);
    ui.Q6Left->setSingleStep(0.1);
    ui.XHead->setRange(0.0, 9.0);
    ui.XHead->setSingleStep(0.1);
    ui.YHead->setRange(0.0, 9.0);
    ui.YHead->setSingleStep(0.1);
    ui.ZHead->setRange(0.0, 2.0);
    ui.ZHead->setSingleStep(0.1);

    actionClientSup_.waitForServer();
    actionClientTorso_.waitForServer();
    actionClientRightArm_.waitForServer();
    actionClientLeftArm_.waitForServer();
    actionClientRightGripper_.waitForServer();
    actionClientLeftGripper_.waitForServer();
    actionClientHead_.waitForServer();
    actionClientGetQ_.waitForServer();


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
    actionClientSup_.sendGoal(goal);
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
    ros::ServiceClient stop = node_.serviceClient<supervisor_msgs::Empty>("action_executor/stop");
    supervisor_msgs::Empty srv;
    if (!stop.call(srv)) {
       ROS_ERROR("Failed to call service action_executor/stop");
       return;
    }
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

bool MainWindow::toIgnore(string fact){
    for(vector<string>::iterator it = toIgnoreFacts.begin(); it != toIgnoreFacts.end(); it++){
        if(fact == *it){
            return true;
        }
    }
    return false;
}


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
           }else if(!toIgnore(it->property)){
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


/*************************************
 * Dialogue tab
 *************************************/

void MainWindow::on_SpeakButton_clicked()
{
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::Say>("dialogue_node/say");
    supervisor_msgs::Say srv;
    string sentence = ui.SpeakSentence->text().toStdString();
    srv.request.sentence = sentence;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service dialogue_node/say");
    }
}


void MainWindow::on_YesButton_clicked()
{
    supervisor_msgs::Bool msg;
    msg.boolAnswer = true;
    boolAnswerPub_.publish(msg);
}

void MainWindow::on_NoButton_clicked()
{
    supervisor_msgs::Bool msg;
    msg.boolAnswer = false;
    boolAnswerPub_.publish(msg);
}

void MainWindow::on_SendFactButton_clicked()
{
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfoDia>("dialogue_node/get_info");
    supervisor_msgs::GetInfoDia srv;
    srv.request.type = "FACT";
    toaster_msgs::Fact fact;
    fact.subjectId = ui.SubjectFact->text().toStdString();
    fact.property = ui.PropertyFact->text().toStdString();
    fact.targetId = ui.TargetFact->text().toStdString();
    srv.request.fact = fact;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service dialogue_node/get_info");
    }
}

/*************************************
 * Robot control tab
 *************************************/

void MainWindow::on_moveTorso_clicked()
{
    pr2motion::Torso_MoveGoal goal;
    goal.torso_position = ui.TorsoValue->value();
    actionClientTorso_.sendGoal(goal);
}

void MainWindow::on_stopTorso_clicked()
{
    ros::ServiceClient client = node_.serviceClient<pr2motion::Torso_Stop>("pr2motion/Torso_Stop");
    pr2motion::Torso_Stop srv;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service pr2motion/Torso_Stop");
    }
}

void MainWindow::on_openRightGripper_clicked()
{
    if(!simu_){
        pr2motion::Gripper_Right_OperateGoal goal;
        goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
        actionClientRightGripper_.sendGoal(goal);
    }
}

void MainWindow::on_closeRightGripper_clicked()
{
    if(!simu_){
        pr2motion::Gripper_Right_OperateGoal goal;
        goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
        actionClientRightGripper_.sendGoal(goal);
    }
}

void MainWindow::on_stopRightGripper_clicked()
{
    if(!simu_){
        ros::ServiceClient client = node_.serviceClient<pr2motion::Gripper_Stop>("pr2motion/Gripper_Stop");
        pr2motion::Gripper_Stop srv;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service pr2motion/Gripper_Stop");
        }
    }
}

void MainWindow::on_openLeftGripper_clicked()
{
    if(!simu_){
        pr2motion::Gripper_Left_OperateGoal goal;
        goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
        actionClientLeftGripper_.sendGoal(goal);
    }
}

void MainWindow::on_closeLeftGripper_clicked()
{
    if(!simu_){
        pr2motion::Gripper_Left_OperateGoal goal;
        goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
        actionClientLeftGripper_.sendGoal(goal);
    }
}

void MainWindow::on_stopLeftGripper_clicked()
{
    if(!simu_){
        ros::ServiceClient client = node_.serviceClient<pr2motion::Gripper_Stop>("pr2motion/Gripper_Stop");
        pr2motion::Gripper_Stop srv;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service pr2motion/Gripper_Stop");
        }
    }
}

void MainWindow::on_moveRightArm_clicked()
{
    pr2motion::Arm_Right_MoveToQGoalGoal goal;
    goal.traj_mode.value = pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_PATH;
    goal.shoulder_pan_joint = ui.Q1Right->value();
    goal.shoulder_lift_joint = ui.Q2Right->value();
    goal.upper_arm_roll_joint = ui.Q3Right->value();
    goal.elbow_flex_joint = ui.Q4Right->value();
    goal.forearm_roll_joint = ui.Q5Right->value();
    goal.wrist_flex_joint = ui.Q6Right->value();
    goal.wrist_roll_joint = ui.Q7Right->value();
    actionClientRightArm_.sendGoal(goal);
}

void MainWindow::on_stopRightArm_clicked()
{
    actionClientRightArm_.cancelGoal();
}

void MainWindow::on_moveLeftArm_clicked()
{
    pr2motion::Arm_Left_MoveToQGoalGoal goal;
    goal.traj_mode.value = pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_PATH;
    goal.shoulder_pan_joint = ui.Q1Left->value();
    goal.shoulder_lift_joint = ui.Q2Left->value();
    goal.upper_arm_roll_joint = ui.Q3Left->value();
    goal.elbow_flex_joint = ui.Q4Left->value();
    goal.forearm_roll_joint = ui.Q5Left->value();
    goal.wrist_flex_joint = ui.Q6Left->value();
    goal.wrist_roll_joint = ui.Q7Left->value();
    actionClientLeftArm_.sendGoal(goal);
}

void MainWindow::on_stopLeftArm_clicked()
{
    actionClientLeftArm_.cancelGoal();
}

void MainWindow::on_moveHead_clicked()
{
    pr2motion::Head_MoveGoal goal;
    goal.head_mode.value = pr2motion::pr2motion_HEAD_MODE::pr2motion_HEAD_LOOKAT;
    goal.head_target_frame = "base_link";
    goal.head_target_x = ui.XHead->value();
    goal.head_target_y = ui.YHead->value();
    goal.head_target_z = ui.ZHead->value();
    actionClientHead_.sendGoal(goal);
}

/**

 **/
#ifndef graphical_interface_MAIN_WINDOW_H
#define graphical_interface_MAIN_WINDOW_H

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QMessageBox>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "ui_main_window.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <pr2motion/Arm_Right_MoveToQGoalAction.h>
#include <pr2motion/Arm_Left_MoveToQGoalAction.h>
#include <pr2motion/Gripper_Right_OperateAction.h>
#include <pr2motion/Gripper_Left_OperateAction.h>
#include <pr2motion/Torso_MoveAction.h>
#include <pr2motion/Head_Move_TargetAction.h>
#include <pr2motion/GetQAction.h>
#include <pr2motion/Torso_Stop.h>
#include <pr2motion/Gripper_Stop.h>

#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ActionExecutorActionResult.h"
#include "supervisor_msgs/ActionExecutorActionFeedback.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/HumanActionSimu.h"
#include "supervisor_msgs/GetInfo.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/Empty.h"
#include "toaster_msgs/Fact.h"
#include "supervisor_msgs/ActionMS.h"
#include "supervisor_msgs/Say.h"
#include "supervisor_msgs/GetInfoDia.h"
#include "supervisor_msgs/Bool.h"

using namespace std;

typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> ClientSup;
typedef actionlib::SimpleActionClient<pr2motion::Torso_MoveAction> ClientTorso;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveToQGoalAction> ClientRightArm;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveToQGoalAction> ClientLeftArm;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction> ClientRightGripper;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction> ClientLeftGripper;
typedef actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction> ClientHead;
typedef actionlib::SimpleActionClient<pr2motion::GetQAction> ClientGetQ;

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:

private Q_SLOTS:
    void on_pushButtonExecuteAction_clicked();

    void on_pushButtonAskAction_clicked();

    void on_pushButtonSendHumanAction_clicked();

    void on_pushButtonExecuteGoal_clicked();

    void on_pushButtonPrintKnowledge_clicked();

    void on_pushButtonSeeActions_clicked();

    void on_pushButtonStopAction_clicked();

    bool toIgnore(string fact);

    void on_SpeakButton_clicked();


    void on_moveTorso_clicked();

    void on_stopTorso_clicked();

    void on_openRightGripper_clicked();

    void on_closeRightGripper_clicked();

    void on_stopRightGripper_clicked();

    void on_openLeftGripper_clicked();

    void on_closeLeftGripper_clicked();

    void on_stopLeftGripper_clicked();

    void on_moveRightArm_clicked();

    void on_stopRightArm_clicked();

    void on_moveLeftArm_clicked();

    void on_stopLeftArm_clicked();

    void on_moveHead_clicked();

    void on_YesButton_clicked();

    void on_NoButton_clicked();

    void on_SendFactButton_clicked();

public:
    Ui::MainWindowDesign ui;
    ros::NodeHandle node_;
    ClientSup actionClientSup_;
    ClientTorso actionClientTorso_;
    ClientRightArm actionClientRightArm_;
    ClientLeftArm actionClientLeftArm_;
    ClientRightGripper actionClientRightGripper_;
    ClientLeftGripper actionClientLeftGripper_;
    ClientHead actionClientHead_;
    ClientGetQ actionClientGetQ_;
    string robotName_;
    ros::Publisher boolAnswerPub_;

private:
    double waitActionServer_;
    bool simu_;


};


#endif // graphical_interface_MAIN_WINDOW_H

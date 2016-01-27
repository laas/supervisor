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

#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ActionExecutorActionResult.h"
#include "supervisor_msgs/ActionExecutorActionFeedback.h"
#include "supervisor_msgs/ActionState.h"
#include "supervisor_msgs/HumanActionSimu.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/GetFactsAgent.h"
#include "supervisor_msgs/GetActions.h"
#include "toaster_msgs/Fact.h"
#include "supervisor_msgs/ActionMS.h"

using namespace std;

typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;

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

private:
    Ui::MainWindowDesign ui;
    ros::NodeHandle node_;
    Client actionClient_;
    string robotName_;
};


#endif // graphical_interface_MAIN_WINDOW_H

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

#include "std_srvs/Empty.h"

#include "toaster_msgs/SetInfoDB.h"
#include "toaster_msgs/GetInfoDB.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/RemoveFromHand.h"

#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/HumanAction.h"

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:

private Q_SLOTS:

    void on_pushButtonAddDB_clicked();

    void on_pushButtonRmDB_clicked();

    void on_pushButtonPrintDB_clicked();

    void on_pushButtonPrintAllDB_clicked();

    void on_pushButtonExecuteAction_clicked();

    void on_pushButtonStopAction_clicked();

    void on_pushButtonDetach_clicked();

    void on_pushButtonExecuteHuman_clicked();

    void on_pushButtonDetachHuman_clicked();

public:
    Ui::MainWindowDesign ui; /**< Main windows design*/
    ros::NodeHandle node_; /**< Node handle*/

private:
    double waitActionServer_; /**< time to wait for an action server*/
    bool simu_; /**< flag to indicate simu or not*/
    std::string robotName_; /**< name of the robot*/

    bool databaseTab_; /**< flag to indicate the database tab is activated*/
    bool actionTab_; /**< flag to indicate the action tab is activated*/
    bool humanTab_; /**< flag to indicate the human action tab is activated*/

    ros::ServiceClient client_set_db_; /**< client of set info db service*/
    ros::ServiceClient client_get_db_; /**< client of get info db service*/
    ros::ServiceClient client_execute_db_; /**< client of execute db service*/
    ros::ServiceClient client_stop_action_; /**< client to stop an action*/
    ros::ServiceClient client_detach_object_; /**< client to detach an object from hand*/
    ros::ServiceClient client_human_action_; /**< client to execute human action*/

    actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> actionClient_; /**< action executor client*/
};


#endif // graphical_interface_MAIN_WINDOW_H

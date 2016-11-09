#include <graphical_interface/mainwindow.h>


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent)
{
    ui.setupUi(this);
    setWindowTitle("PR2 supervisor");
    objectInHand_ = "NULL";
    human_client_ = node_.serviceClient<supervisor_msgs::HumanAction>("human_monitor/human_action_simu");

}

MainWindow::~MainWindow()
{
}


void MainWindow::on_pushButtonPickRed_clicked(){

    ROS_INFO("Human picks the red cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("RED_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "RED_CUBE";
}


void MainWindow::on_pushButtonPickGreen_clicked(){

    ROS_INFO("Human picks the green cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("GREEN_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "GREEN_CUBE";
}


void MainWindow::on_pushButtonPickBlue_clicked(){

    ROS_INFO("Human picks the blue cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("BLUE_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "BLUE_CUBE";
}


void MainWindow::on_pushButtonPickBlack_clicked(){

    ROS_INFO("Human picks the black cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("BLACK_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "BLACK_CUBE";
}


void MainWindow::on_pushButtonPlaceRed_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the red cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("RED_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlaceGreen_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the green cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("GREEN_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlaceBlue_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the blue cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("BLUE_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlaceBlack_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the black cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("BLACK_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}

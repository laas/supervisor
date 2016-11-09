#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QMessageBox>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "ui_mainwindow.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include "supervisor_msgs/HumanAction.h"

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:

private Q_SLOTS:

    void on_pushButtonPickRed_clicked();

    void on_pushButtonPickGreen_clicked();

    void on_pushButtonPickBlue_clicked();

    void on_pushButtonPickBlack_clicked();

    void on_pushButtonPlaceRed_clicked();

    void on_pushButtonPlaceGreen_clicked();

    void on_pushButtonPlaceBlue_clicked();

    void on_pushButtonPlaceBlack_clicked();

public:
    Ui::MainWindow ui;

private:
    ros::NodeHandle node_;
    std::string objectInHand_;
    ros::ServiceClient human_client_;

};

#endif // MAINWINDOW_H

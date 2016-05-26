/**
 Main file of the graphical interface
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/graphical_interface/main_window.hpp"



/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    ROS_INFO("[graphical_interface] Init graphical_interface");

    ros::init(argc, argv, "graphical_interface");

    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    ROS_INFO("[graphical_interface] graphical_interface ready");


    int result = app.exec();
    return result;
}

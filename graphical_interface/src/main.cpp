/**
 Main file of the graphical interface
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include <graphical_interface/mainwindow.h>

ros::NodeHandle* node_;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    ROS_INFO("[graphical_interface] Init graphical_interface");

    ros::init(argc, argv, "graphical_interface");
    ros::NodeHandle node;
    ros::Rate loop_rate(30);
    node_ = &node;

    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    ROS_INFO("[graphical_interface] graphical_interface ready");


    int result = app.exec();
    return result;
}

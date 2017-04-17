#ifndef SCAN_H
#define SCAN_H

#include "action_executor/virtual_action.h"
#include "action_executor/Actions/moveTo.h"

#include "std_msgs/String.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class Scan: public VirtualAction{
public:
    Scan(supervisor_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    double timeScan_; /**< time to scan an object*/
    double timeWaitScan_; /**< time to wait for the head focus on the object*/
    ros::ServiceClient client_light_; /**< service client to control the robot light*/
    ros::Subscriber sub_head_focus_; /**< subscriber to the head focus*/
    std::string headFocus_; /**< focus of the robot head*/

    void controlRobotLight(bool on);
    void focusCallback(const std_msgs::String::ConstPtr& msg);

};

#endif // SCAN_H

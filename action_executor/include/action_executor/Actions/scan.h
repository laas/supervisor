#ifndef SCAN_H
#define SCAN_H

#include "action_executor/virtual_action.h"
#include "action_executor/Actions/moveTo.h"

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
    ros::ServiceClient client_light_; /**< service client to control the robot light*/

    void controlRobotLight(bool on);

};

#endif // SCAN_H

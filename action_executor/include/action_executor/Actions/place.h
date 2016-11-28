#ifndef PLACE_H
#define PLACE_H

#include "action_executor/virtual_action.h"

class Place: public VirtualAction{
public:
    Place(supervisor_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    std::string support_; /**< support where to place the object*/
    std::string replacementSupport_; /**< replacement support to use for planning*/


};

#endif // PLACE_H

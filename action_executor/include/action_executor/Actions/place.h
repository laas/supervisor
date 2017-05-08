#ifndef PLACE_H
#define PLACE_H

#include "action_executor/virtual_action.h"

class Place: public VirtualAction{
public:
    Place(supervisor_msgs::Action action, Connector* connector);
    Place() {}
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();

    std::string support_; /**< support where to place the object*/
    std::string initialSupport_; /**< given high level support when there is one*/
    std::string replacementSupport_; /**< replacement support to use for planning*/
protected:

private:


};

#endif // PLACE_H

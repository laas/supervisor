#ifndef PICK_H
#define PICK_H

#include "action_executor/virtual_action.h"


class Pick: public VirtualAction{
public:
    Pick(supervisor_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:


};

#endif // PICK_H

#ifndef MOVETO_H
#define MOVETO_H

#include "action_executor/virtual_action.h"

class MoveTo: public VirtualAction{
public:
    MoveTo(supervisor_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    std::string position_; /**< position to reach*/
    std::string arm_; /**< arm to move*/


};

#endif // MOVETO_H

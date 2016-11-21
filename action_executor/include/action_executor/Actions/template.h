#ifndef TEMPLATE_H
#define TEMPLATE_H

#include "action_executor/virtual_action.h"

//Replace here Template by the new action name
class Template: public VirtualAction{
public:
    Template(supervisor_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    //add here specifics parameters for the action


};

#endif // TEMPLATE_H

#ifndef PLACEREACHABLE_H
#define PLACEREACHABLE_H

#include "action_executor/virtual_action.h"

class PlaceReachable: public VirtualAction{
public:
    PlaceReachable(supervisor_msgs::Action action, Connector* connector);
    PlaceReachable() {}
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    std::string support_; /**< support where to place the object*/
    std::string initialSupport_; /**< given high level support when there is one*/
    std::string targetAgent_; /**< agent the object should be reachable by*/
    std::string replacementSupport_; /**< replacement support to use for planning*/


};

#endif // PLACEREACHABLE_H

#ifndef PICKANDPLACEREACHABLE_H
#define PICKANDPLACEREACHABLE_H

#include "action_executor/virtual_action.h"
#include "action_executor/Actions/pick.h"
#include "action_executor/Actions/placeReachable.h"

class PickAndPlaceReachable: public VirtualAction{
public:
    PickAndPlaceReachable(supervisor_msgs::Action action, Connector* connector);
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    Pick pickAction_; /**< pick part of the action*/
    PlaceReachable placeAction_; /**< place part of the action*/
    std::string support_; /**< support where to place the object*/


};

#endif // PICKANDPLACEREACHABLE_H

#ifndef DROP_H
#define DROP_H

#include "action_executor/virtual_action.h"

class Drop: public VirtualAction{
public:
    Drop(supervisor_msgs::Action action, Connector* connector);
    Drop() {}
    virtual bool preconditions();
    virtual bool plan();
    virtual bool exec(Server* action_server);
    virtual bool post();
protected:

private:
    std::string container_; /**< container where to drop the object*/
    std::string initialContainer_; /**< given high level container when there is one*/


};

#endif // DROP_H

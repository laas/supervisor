#ifndef VIRTUALACTION_H
#define VIRTUALACTION_H

#include "action_executor/connector.h"

class VirtualAction{
public:
    VirtualAction(Connector* connector);
    ~VirtualAction() {}
    virtual bool preconditions() = 0;
    virtual bool plan() = 0;
    virtual bool exec(Server* action_server) = 0;
    virtual bool post() = 0;

    void moveRightArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Right_MoveResultConstPtr& result);
    void moveLeftArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Left_MoveResultConstPtr& result);
    void moveRightGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Right_OperateResultConstPtr& result);
    void moveLeftGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Left_OperateResultConstPtr& result);
protected:
   Connector* connector_; /**< connector structure pointer*/

   std::string object_; /**< object of the action*/
   bool gripperEmpty_; /**< indicates if the gripper is empty after closing it*/
   int gtpActionId_; /**< id of the gtp task*/
   std::vector<gtp_ros_msgs::SubSolution> subSolutions_; /**< subSolutions of the gtp task*/

   bool isManipulableObject(std::string object);
   bool isSupportObject(std::string support);
   bool isContainerObject(std::string container);
   bool ArePreconditionsChecked(std::vector<toaster_msgs::Fact> precs);
   void PutInHand(std::string object, std::string hand, int gtpId);
   void RemoveFromHand(std::string object);
   void PutOnSupport(std::string object, std::string support);
   void PutInContainer(std::string object, std::string container);
   std::pair<int, std::vector<gtp_ros_msgs::SubSolution> >  planGTP(std::string actionName, std::vector<gtp_ros_msgs::Role> agents, std::vector<gtp_ros_msgs::Role> objects, std::vector<gtp_ros_msgs::MiscData> datas, std::vector<gtp_ros_msgs::Point> points);
   bool execAction(int actionId, std::vector<gtp_ros_msgs::SubSolution> subSolutions, bool shouldOpen, Server* action_server);
   bool executeTrajectory(int actionId, int actionSubId, int armId, Server* action_server);
   bool openGripper(int armId, Server* action_server);
   bool closeGripper(int armId, Server* action_server);
   bool isGripperEmpty(std::string arm);
private:

   ros::ServiceClient client_db_execute_; /**< toaster client for database execute commands*/
   ros::ServiceClient client_put_hand_; /**< toaster client to put objects in hand*/
   ros::ServiceClient client_remove_hand_; /**< toaster client to remove objects from hand*/
   ros::ServiceClient client_set_pose_; /**< toaster client to set entities poses*/
   ros::ServiceClient client_gtp_traj_; /**< gtp client to publish a traj*/
};

#endif // VIRTUALACTION_H

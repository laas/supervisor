#include <graphical_interface/mainwindow.h>


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent)
{
    ui.setupUi(this);
    setWindowTitle("PR2 supervisor");
    objectInHand_ = "NULL";
    human_client_ = node_.serviceClient<supervisor_msgs::HumanAction>("human_monitor/human_action_simu");

}

MainWindow::~MainWindow()
{
}


void MainWindow::on_pushButtonPickRed_clicked(){

    ROS_INFO("Human picks the red cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("RED_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "RED_CUBE";
}


void MainWindow::on_pushButtonPickGreen_clicked(){

    ROS_INFO("Human picks the green cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("GREEN_CUBE2");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "GREEN_CUBE2";
}


void MainWindow::on_pushButtonPickBlue_clicked(){

    ROS_INFO("Human picks the blue cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("BLUE_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "BLUE_CUBE";
}


void MainWindow::on_pushButtonPickBlack_clicked(){

    ROS_INFO("Human picks the black cube");

    //create the action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back("BLACK_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "BLACK_CUBE";
}


void MainWindow::on_pushButtonPlaceRed_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the red cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("RED_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlaceGreen_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the green cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("GREEN_CUBE2");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlaceBlue_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the blue cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("BLUE_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlaceBlack_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the black cube", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("BLACK_CUBE");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonPlacePlacemat_clicked(){

    if(objectInHand_ == "NULL"){
        ROS_WARN("No object in hand!");
        return;
    }

    ROS_INFO("Human places %s on the placemat red", objectInHand_.c_str());

    //create the action
    supervisor_msgs::Action action;
    action.name = "place";
    action.actors.push_back("HERAKLES_HUMAN1");
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(objectInHand_);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back("PLACEMAT_RED");


    //Call the human monitor
    supervisor_msgs::HumanAction srv;
    srv.request.action = action;
    srv.request.agent = "HERAKLES_HUMAN1";
    if (!human_client_.call(srv)){
     ROS_ERROR("[graphical_interface] Failed to call service human_monitor/human_action_simu");
    }

    objectInHand_ = "NULL";
}


void MainWindow::on_pushButtonSetEnv_clicked(){

    ros::ServiceClient client = node_.serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    toaster_msgs::SetEntityPose srv;
    double x, y, z;

    toaster_msgs::ObjectListStamped objectList;
    std::string objectRef;
    double xRef,yRef,zRef;
    node_.getParam("environment/referenceObject", objectRef);
    bool found = false;
    objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
     if(it->meEntity.id == objectRef){
        xRef = it->meEntity.pose.position.x;
        yRef = it->meEntity.pose.position.y;
        zRef = it->meEntity.pose.position.z;
        found = true;
        break;
     }
    }
    if(!found){
       ROS_ERROR("No reference object position!");
       return;
    }
    //get list of entities to move with pdg
    std::string pdgTopic = "/environment/pdgObjects";
    std::vector<std::string> pdgObjects;
    node_.getParam(pdgTopic, pdgObjects);
    for(std::vector<std::string>::iterator it = pdgObjects.begin(); it != pdgObjects.end(); it++){
           std::string posex = "/environment/objectsRelativePose/" + *it + "/x" ;
           std::string posey = "/environment/objectsRelativePose/" + *it + "/y" ;
           std::string posez = "/environment/objectsRelativePose/" + *it + "/z" ;
           std::string rotationTopic = "/environment/objectsRelativePose/" + *it + "/rotation" ;
           double xRel, yRel, zRel;
           bool rotation;
           node_.getParam(posex, xRel);
           node_.getParam(posey, yRel);
           node_.getParam(posez, zRel);
           node_.getParam(rotationTopic, rotation);
           srv.request.id = *it;
           srv.request.type = "object";
           srv.request.pose.position.x = xRef + xRel;
           srv.request.pose.position.y = yRef + yRel;
           srv.request.pose.position.z = zRel;
           srv.request.pose.orientation.x = 0.0;
           srv.request.pose.orientation.y = 0.0;
           if(rotation){
               srv.request.pose.orientation.z = 0.7;
               srv.request.pose.orientation.w = 0.7;
           }else{
               srv.request.pose.orientation.z = 0.0;
               srv.request.pose.orientation.w = 1.0;
           }
           if (!client.call(srv)){
             ROS_ERROR("Failed to call service pdg/set_entity_pose");
            }
    }
}


//
// Created by phlf on 24/03/16.
//
#include <iostream>
#include <string>
#include <boost/timer.hpp>
#include <cafer_core/cafer_core.hpp>
#include <dream_babbling/move_baxter_arm.h>
#include <dream_babbling/pose_goalAction.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/callback_queue.h>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/timer.hpp>

using namespace dream_babbling;
using namespace cafer_core;

class Controller: public Component{
using Component::Component; // C++11 requirement to inherit the constructor

public :

void client_connect_to_ros(){
    XmlRpc::XmlRpcValue glob_params;
    std::stringstream display_params;

    cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
    for (auto& param:glob_params) {
        display_params << "\t" << param.first << ":\t" << param.second << std::endl;
    }

    ROS_INFO_STREAM("CONTROLLER : Global parameters retrieved:" << std::endl << display_params.str());

    _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, glob_params["controller_server"],
                                                                   boost::bind(&Controller::execute, this, _1),false));
    _move_baxter_arm.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<dream_babbling::move_baxter_arm>("move_baxter_arm")));
}

void init(){
    connect_to_ros();
    _serv->start();
    _is_init = true;
    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();
    usleep(2e6);
}

void execute(const pose_goalGoalConstPtr& poseGoal)
{

    if (true){
        ROS_INFO_STREAM("CONTROLLER : home is reached and the robot is ready for next iteration");
        _serv->setSucceeded();
    }
    else{
        _serv->setAborted();
    }
}

void client_disconnect_from_ros(){}
void update(){}

private:
std::shared_ptr<actionlib::SimpleActionServer<pose_goalAction>> _serv;
std::unique_ptr<ros::ServiceClient> _move_baxter_arm;
Eigen::VectorXd _pose_home;
dream_babbling::pose_goalFeedback _joints_pose_feedback;
};

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
std::string key;
std::string value;
std::string temp_str;
std::string::size_type res;

key = "__name:=";

for (unsigned short i = 0; i < argc; ++i) {
    temp_str = argv[i];
    res = temp_str.find(key);

    if (res != std::string::npos) {
        value = temp_str.erase(res, key.length());
        break;
    }
    else if (i == argc - 1) {
        value = default_val;
    }
}
return value;
}

int main(int argc, char** argv){
std::string node_name;
XmlRpc::XmlRpcValue cafer;

node_name = parse_arg(argc, argv, "controller_node");

cafer_core::init(argc, argv, node_name);
cafer_core::ros_nh->getParam("cafer", cafer);
ROS_INFO_STREAM(cafer_core::ros_nh->getNamespace());

Controller controller(cafer["mgmt"], cafer["type"], cafer["freq"],cafer["uuid"]);

controller.wait_for_init();
controller.spin();

ROS_INFO_STREAM("CONTROLLER : Robot controller ready !");

while (ros::ok() && (!controller.get_terminate())) {
    controller.spin();
}
return 0;
}

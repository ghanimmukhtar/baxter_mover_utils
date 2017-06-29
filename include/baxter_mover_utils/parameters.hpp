#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>
#include <Eigen/Core>

#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <std_srvs/Empty.h>

#include <tf/tf.h>

struct Parameters {
    std::vector<std::string> arm_joints_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    sensor_msgs::JointState my_joint_state;
    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    geometry_msgs::PoseStamped pose_target;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;

    XmlRpc::XmlRpcValue planner_parameters;

    moveit_msgs::GetMotionPlanRequest final_motion_request;
    moveit_msgs::GetMotionPlanResponse final_motion_response;

    moveit_msgs::ExecuteKnownTrajectoryRequest execute_traj_req;
    moveit_msgs::ExecuteKnownTrajectoryResponse execute_traj_res;

    std_srvs::Empty::Request empty_octomap_request;
    std_srvs::Empty::Response empty_octomap_response;

    std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader;
    robot_model::RobotModelPtr robot_model;
};

class Data_config{

public:
    Parameters params;

    Data_config(){

    }

    //// Getters
    //left and right grippers pose variables getters
    geometry_msgs::Pose& get_eef_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_pose;
        else
            return params.r_eef_pose;
    }

    geometry_msgs::PoseStamped& get_pose_target(){
        return params.pose_target;
    }

    std::vector<std::string>& get_baxter_arm_joints_names(){
        return params.arm_joints_names;
    }

    sensor_msgs::JointState& get_joint_state(){
        return params.my_joint_state;
    }

    Eigen::VectorXd& get_eef_rpy_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_rpy_pose;
        else
            return params.r_eef_rpy_pose;
    }

    Eigen::Vector3d& get_eef_position(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_position;
        else
            return params.r_eef_position;
    }

    Eigen::Vector3d& get_eef_rpy_orientation(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return params.l_eef_rpy_orientation;
        else
            return params.r_eef_rpy_orientation;
    }

    XmlRpc::XmlRpcValue& get_planner_parameters(){
        return params.planner_parameters;
    }

    moveit_msgs::GetMotionPlanRequest& get_motion_request(){
        return params.final_motion_request;
    }

    moveit_msgs::GetMotionPlanResponse& get_motion_response(){
        return params.final_motion_response;
    }

    moveit_msgs::ExecuteKnownTrajectoryRequest& get_motion_execute_request(){
        return params.execute_traj_req;
    }

    moveit_msgs::ExecuteKnownTrajectoryResponse& get_motion_execute_response(){
        return params.execute_traj_res;
    }

    std_srvs::Empty::Request& get_empty_octomap_request(){
        return params.empty_octomap_request;
    }

    std_srvs::Empty::Response& get_empty_octomap_response(){
        return params.empty_octomap_response;
    }
    //// Setters
    //left and right grippers pose variables getters
    void set_robot_model_loader(){
        params.robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    }

    void set_robot_model(){
        params.robot_model = params.robot_model_loader->getModel();
    }

    void set_joint_state(const sensor_msgs::JointState::ConstPtr& jo_state){
        params.my_joint_state = *jo_state;
    }

    void set_eef_pose(geometry_msgs::Pose& eef_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_pose = eef_pose;
        else
            params.r_eef_pose = eef_pose;
    }

    void set_pose_target(geometry_msgs::PoseStamped& pose_target){
        params.pose_target = pose_target;
    }

    void set_eef_rpy_pose(Eigen::VectorXd& eef_rpy_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_rpy_pose = eef_rpy_pose;
        else
            params.r_eef_rpy_pose = eef_rpy_pose;
    }

    void set_eef_position(Eigen::Vector3d& eef_position, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_position = eef_position;
        else
            params.r_eef_position = eef_position;
    }

    void set_eef_rpy_orientation(Eigen::Vector3d& eef_rpy_orientation, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_rpy_orientation = eef_rpy_orientation;
        else
            params.r_eef_rpy_orientation = eef_rpy_orientation;
    }
    void set_planner_parameters(XmlRpc::XmlRpcValue& planner_params){
        params.planner_parameters = planner_params;
    }

    void set_motion_request(moveit_msgs::GetMotionPlanRequest& motion_plan_request){
        params.final_motion_request = motion_plan_request;
    }

    void set_motion_response(moveit_msgs::GetMotionPlanResponse& motion_plan_response){
        params.final_motion_response = motion_plan_response;
    }
};

#endif

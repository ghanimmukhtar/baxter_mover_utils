#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>
#include <Eigen/Core>

#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <tf/tf.h>

struct Parameters {
    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;
    XmlRpc::XmlRpcValue planner_parameters;
    moveit_msgs::GetMotionPlanRequest final_motion_request;
    moveit_msgs::GetMotionPlanResponse final_motion_response;
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

    //// Setters
    //left and right grippers pose variables getters
    void set_eef_pose(geometry_msgs::Pose& eef_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            params.l_eef_pose = eef_pose;
        else
            params.r_eef_pose = eef_pose;
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

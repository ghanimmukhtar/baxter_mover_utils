#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <Eigen/Core>

#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

#include <aruco/aruco.h>

#include <tf/tf.h>

struct Parameters {
    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;
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

};

#endif

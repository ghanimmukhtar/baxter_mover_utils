#include <ros/ros.h>
#include <dream_babbling/move_baxter_arm.h>
#include <baxter_core_msgs/EndpointState.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

void locate_eef_pose(geometry_msgs::Pose &eef_feedback, const std::string gripper){
    Eigen::VectorXd end_effector_pose(6);
    geometry_msgs::Pose eef_pose_quat = eef_feedback;
    tf::Quaternion eef_rpy_orientation;

    tf::quaternionMsgToTF(eef_pose_quat.orientation, eef_rpy_orientation);

    double roll, yaw, pitch;
    tf::Matrix3x3 m(eef_rpy_orientation);
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d eef_current_position;
    Eigen::Vector3d eef_current_orientation;
    eef_current_position << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z;

    eef_current_orientation <<    roll,
            pitch,
            yaw;
    end_effector_pose << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z,
            roll,
            pitch,
            yaw;
    ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_current_position << "\n and for orientation: " << eef_current_orientation);
}


//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, "left_gripper");
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
}

//call back that register baxter right end effector pose and rearrange the orientation in RPY
void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, "right_gripper");
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
}

bool move_baxter_arm(dream_babbling::move_baxter_arm::Request &req,
                               dream_babbling::move_baxter_arm::Response &res,
                               ros::NodeHandle& nh){
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);

    // Required to trigger the previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    planning_interface::MotionPlanRequest motion_req;
    planning_interface::MotionPlanResponse motion_res;
    if(strcmp(req.arm, "left") == 0)
        motion_req.group_name = "left_arm";

    return true;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "baxter_arm_controller_service");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService<dream_babbling::move_baxter_arm::Request,
                                    dream_babbling::move_baxter_arm::Response>(
                                    "move_baxter_arm",
                                    boost::bind(move_baxter_arm, _1, _2, nh));

    ROS_INFO("Ready to move left or right arm to any position or pose.");

    // Required to publish this service
    ros::spin();
    return 0;
}

#include <baxter_mover_utils/baxter_mover.hpp>

using namespace baxter_mover;

void BAXTER_Mover::init(){
    _baxter_mover.reset(new ros::ServiceServer(nh_->advertiseService("move_baxter_arm", &BAXTER_Mover::move_baxter_arm_cb, this)));
    _get_motion_plan.reset(new ros::ServiceClient(nh_->serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path", 1)));
    _sub_l_eef_msg.reset(new ros::Subscriber(nh_->subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER_Mover::left_eef_Callback, this)));
    _sub_r_eef_msg.reset(new ros::Subscriber(nh_->subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER_Mover::right_eef_Callback, this)));
}

bool BAXTER_Mover::move_baxter_arm_cb(baxter_mover_utils::move_baxter_arm::Request &req,
                                      baxter_mover_utils::move_baxter_arm::Response &res){
    planning_interface::MotionPlanRequest motion_req;
    planning_interface::MotionPlanResponse motion_res;
    moveit_msgs::GetMotionPlanRequest final_motion_request;
    moveit_msgs::GetMotionPlanResponse final_motion_response;
    std::string eef_name;
    geometry_msgs::PointStamped position_target;
    geometry_msgs::PoseStamped pose_target;
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    if(strcmp(req.arm.c_str(), "left") == 0){
        motion_req.group_name = "left_arm";
        eef_name = "left_gripper";
    }
    else{
        motion_req.group_name = "right_arm";
        eef_name = "right_gripper";
    }

    if(strcmp(req.type.c_str(), "position") == 0){
        position_target.header.frame_id = "base";
        position_target.point.x = req.goal[0];
        position_target.point.y = req.goal[1];
        position_target.point.z = req.goal[2];
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, position_target);
        motion_req.goal_constraints.push_back(pose_goal);

    }
    else if(strcmp(req.type.c_str(), "pose") == 0){
        pose_target.header.frame_id = "base";
        pose_target.pose.position.x = req.goal[0];
        pose_target.pose.position.y = req.goal[1];
        pose_target.pose.position.z = req.goal[2];
        pose_target.pose.orientation.x = req.goal[3];
        pose_target.pose.orientation.y = req.goal[4];
        pose_target.pose.orientation.z = req.goal[5];
        pose_target.pose.orientation.w = req.goal[6];
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, pose_target, tolerance_pose, tolerance_angle);
        motion_req.goal_constraints.push_back(pose_goal);
    }
    else if(strcmp(req.type.c_str(), "keep_orientation") == 0){
        pose_target.header.frame_id = "base";
        pose_target.pose = _global_parameters.get_eef_pose(motion_req.group_name);
        pose_target.pose.position.x = req.goal[0];
        pose_target.pose.position.y = req.goal[1];
        pose_target.pose.position.z = req.goal[2];
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, pose_target, tolerance_pose, tolerance_angle);
        motion_req.goal_constraints.push_back(pose_goal);
    }
    else
        ROS_INFO("determine if the target is pose, position or position with same orientation as start");

    final_motion_request.motion_plan_request = motion_req;
    _get_motion_plan->call(final_motion_request, final_motion_response);
    return true;
}

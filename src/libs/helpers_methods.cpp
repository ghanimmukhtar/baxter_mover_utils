#include <baxter_mover_utils/helpers_methods.hpp>

//a method that sets positions/orientations variables in the parameter class
void baxter_helpers_methods::locate_eef_pose(geometry_msgs::Pose &eef_feedback, Data_config& parameters, const std::string gripper){

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
    parameters.set_eef_position(eef_current_position, gripper);
    parameters.set_eef_rpy_orientation(eef_current_orientation, gripper);
    parameters.set_eef_pose(eef_pose_quat, gripper);
    parameters.set_eef_rpy_pose(end_effector_pose, gripper);
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_current_position << "\n and for orientation: " << eef_current_orientation);
}

//setup correctly the motion plan request which will be sent to the move group
void baxter_helpers_methods::prepare_motion_request(baxter_mover_utils::move_baxter_arm::Request &req,
                            baxter_mover_utils::move_baxter_arm::Response &res,
                            Data_config& params){
    ROS_INFO("I am at the service to move the arm");
    params.get_motion_request().motion_plan_request.goal_constraints.clear();
    std::string eef_name;
    geometry_msgs::PointStamped position_target;
    geometry_msgs::PoseStamped pose_target;
    std::vector<double> tolerance_pose(3,  std::stod(params.get_planner_parameters()["tolerance_pose"]));
    std::vector<double> tolerance_angle(3, std::stod(params.get_planner_parameters()["tolerance_angle"]));

    params.get_motion_request().motion_plan_request.planner_id = static_cast<std::string>(params.get_planner_parameters()["planner_id"]);
    params.get_motion_request().motion_plan_request.num_planning_attempts = std::stoi(params.get_planner_parameters()["planning_attempts"]);
    params.get_motion_request().motion_plan_request.allowed_planning_time = std::stod(params.get_planner_parameters()["planning_time"]);

    ROS_INFO_STREAM("planner id is: " << params.get_motion_request().motion_plan_request.planner_id);
    ROS_INFO_STREAM("planner attempts is: " << params.get_motion_request().motion_plan_request.num_planning_attempts);
    ROS_INFO_STREAM("planner time is: " << params.get_motion_request().motion_plan_request.allowed_planning_time);

    if(strcmp(req.arm.c_str(), "left") == 0){
        params.get_motion_request().motion_plan_request.group_name = "left_arm";
        eef_name = "left_gripper";
    }
    else{
        ROS_INFO_STREAM("arm chosing is: " << req.arm);
        params.get_motion_request().motion_plan_request.group_name = "right_arm";
        eef_name = "right_gripper";
    }

    if(strcmp(req.type.c_str(), "position") == 0){
        ROS_INFO("building the position item regardless of orientation");
        position_target.header.frame_id = "/base";
        position_target.point.x = req.goal[0];
        position_target.point.y = req.goal[1];
        position_target.point.z = req.goal[2];
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, position_target);
        ROS_WARN_STREAM("the goal is: "
                << req.goal[0] << ", "
                << req.goal[1] << ", "
                << req.goal[2]);
        params.get_motion_request().motion_plan_request.goal_constraints.push_back(pose_goal);

    }
    else if(strcmp(req.type.c_str(), "pose") == 0){
        ROS_INFO("building the pose item");
        pose_target.header.frame_id = "/base";
        pose_target.pose.position.x = req.goal[0];
        pose_target.pose.position.y = req.goal[1];
        pose_target.pose.position.z = req.goal[2];
        pose_target.pose.orientation.x = req.goal[3];
        pose_target.pose.orientation.y = req.goal[4];
        pose_target.pose.orientation.z = req.goal[5];
        pose_target.pose.orientation.w = req.goal[6];
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, pose_target, tolerance_pose, tolerance_angle);
        params.get_motion_request().motion_plan_request.goal_constraints.push_back(pose_goal);
    }
    else if(strcmp(req.type.c_str(), "keep_orientation") == 0){
        ROS_INFO("building the position item and trying to keep orientation as start");
        pose_target.header.frame_id = "/base";
        pose_target.pose = params.get_eef_pose(eef_name);
        pose_target.pose.position.x = req.goal[0];
        pose_target.pose.position.y = req.goal[1];
        pose_target.pose.position.z = req.goal[2];
        ROS_WARN_STREAM("the goal is: "
                << req.goal[0] << ", "
                << req.goal[1] << ", "
                << req.goal[2] << ", "
                   << pose_target.pose.orientation.w << ", "
                      << pose_target.pose.orientation.x << ", "
                         << pose_target.pose.orientation.y << ", "
                            << pose_target.pose.orientation.z << ", ");
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(eef_name, pose_target, tolerance_pose, tolerance_angle);
        params.get_motion_request().motion_plan_request.goal_constraints.push_back(pose_goal);
    }
    else
        ROS_INFO("determine if the target is pose, position or position with same orientation as start");
}

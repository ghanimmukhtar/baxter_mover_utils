#include <ros/ros.h>

#include <baxter_core_msgs/EndpointState.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <baxter_mover_utils/parameters.hpp>
#include <baxter_mover_utils/move_baxter_arm.h>

namespace baxter_helpers_methods{
void locate_eef_pose(geometry_msgs::Pose &eef_feedback, const std::string gripper);
}

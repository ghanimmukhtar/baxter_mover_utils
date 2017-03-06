#include <baxter_mover_utils/baxter_mover.hpp>

using namespace baxter_mover;

void BAXTER_Mover::init(ros::NodeHandle& nh){
    _baxter_mover.reset(new ros::ServiceServer(nh.advertiseService("move_baxter_arm", &BAXTER_Mover::move_baxter_arm_cb, this)));
    _get_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path", 1)));
    _sub_l_eef_msg.reset(new ros::Subscriber(nh.subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER_Mover::left_eef_Callback, this)));
    _sub_r_eef_msg.reset(new ros::Subscriber(nh.subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER_Mover::right_eef_Callback, this)));

    nh.getParam("planner_parameters", _global_parameters.get_planner_parameters());
    //nh.getParam("planner_id", _planner_id);
    _my_spinner.reset(new ros::AsyncSpinner(1));
    _my_spinner->start();
}

bool BAXTER_Mover::move_baxter_arm_cb(baxter_mover_utils::move_baxter_arm::Request &req,
                                      baxter_mover_utils::move_baxter_arm::Response &res){

    baxter_helpers_methods::prepare_motion_request(req, res, _global_parameters);

    _get_motion_plan->call(_global_parameters.get_motion_request(), _global_parameters.get_motion_response());
    ROS_INFO_STREAM("the response success is: " << _global_parameters.get_motion_response().motion_plan_response.error_code);
    return true;
}

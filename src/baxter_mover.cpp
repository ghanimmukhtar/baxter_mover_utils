#include <baxter_mover_utils/baxter_mover.hpp>

using namespace baxter_mover;

void BAXTER_Mover::init(ros::NodeHandle& nh){
    _baxter_mover.reset(new ros::ServiceServer(nh.advertiseService("move_baxter_arm", &BAXTER_Mover::move_baxter_arm_cb, this)));
    _get_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path", 1)));
    _execute_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path", 1)));
    _sub_l_eef_msg.reset(new ros::Subscriber(nh.subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER_Mover::left_eef_Callback, this)));
    _sub_r_eef_msg.reset(new ros::Subscriber(nh.subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER_Mover::right_eef_Callback, this)));

    ROS_ERROR_STREAM("MY NAME SPACE IS: " << nh.getNamespace() << " /////////////////////////!!!!!!!!!!!!!!");
    nh.getParam(nh.getNamespace() + "/planner_parameters", _global_parameters.get_planner_parameters());
    //nh.getParam("planner_id", _planner_id);
    _my_spinner.reset(new ros::AsyncSpinner(1));
    _my_spinner->start();
}

bool BAXTER_Mover::move_baxter_arm_cb(baxter_mover_utils::move_baxter_arm::Request &req,
                                      baxter_mover_utils::move_baxter_arm::Response &res){

    baxter_helpers_methods::prepare_motion_request(req, res, _global_parameters);



    bool plan_success = false, execute_success = false;
    if(_get_motion_plan->call(_global_parameters.get_motion_request(), _global_parameters.get_motion_response())){
        plan_success = true;
        _global_parameters.get_motion_execute_request().trajectory = _global_parameters.get_motion_response().motion_plan_response.trajectory;
        _global_parameters.get_motion_execute_request().wait_for_execution = true;
        if(_execute_motion_plan->call(_global_parameters.get_motion_execute_request(), _global_parameters.get_motion_execute_response()))
            execute_success = true;
    }
    else
        plan_success = false;
    ROS_INFO_STREAM("the planning response error_code is: " << _global_parameters.get_motion_response().motion_plan_response.error_code);
    ROS_INFO_STREAM("and planning success is: " << plan_success);
    ROS_INFO_STREAM("and executing success is: " << execute_success);

    return execute_success;
}

#include <baxter_mover_utils/baxter_mover.hpp>

using namespace baxter_mover;
using namespace moveit::planning_interface;

void BAXTER_Mover::init(ros::NodeHandle& nh){
    _baxter_mover.reset(new ros::ServiceServer(nh.advertiseService("move_baxter_arm", &BAXTER_Mover::move_baxter_arm_cb, this)));
    _get_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path", 1)));
    _execute_motion_plan.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path", 1)));
    _clear_octomap.reset(new ros::ServiceClient(nh.serviceClient<std_srvs::Empty>("/clear_octomap", 1)));
    _sub_l_eef_msg.reset(new ros::Subscriber(nh.subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER_Mover::left_eef_Callback, this)));
    _sub_r_eef_msg.reset(new ros::Subscriber(nh.subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER_Mover::right_eef_Callback, this)));
    _get_planning_scene.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene", 1)));
    _sub_joint_state_msg.reset(new ros::Subscriber(nh.subscribe("/robot/joint_states", 10, &BAXTER_Mover::joint_state_Callback, this)));
    _psm_pub.reset(new ros::Publisher(nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1)));



    global_parameters.set_robot_model_loader();
    global_parameters.set_robot_model();

    ROS_ERROR_STREAM("MY NAME SPACE IS: " << nh.getNamespace() << " /////////////////////////!!!!!!!!!!!!!!");
    nh.getParam(nh.getNamespace() + "/planner_parameters", global_parameters.get_planner_parameters());

    group.reset(new MoveGroup(MoveGroup::Options(static_cast<std::string>(global_parameters.get_planner_parameters()["babbling_arm"]), MoveGroup::ROBOT_DESCRIPTION, nh)));
    secondary_group.reset(new MoveGroup(MoveGroup::Options(static_cast<std::string>(global_parameters.get_planner_parameters()["secondary_arm"]), MoveGroup::ROBOT_DESCRIPTION, nh)));
    ROS_INFO_STREAM("THE MOVER: The planner id is: " << std::string(global_parameters.get_planner_parameters()["planner_id"]));
    ROS_INFO_STREAM("THE MOVER: The planning time is: " << std::stod(global_parameters.get_planner_parameters()["planning_time"]));
    group->setPlannerId(std::string(global_parameters.get_planner_parameters()["planner_id"]));
    group->setPlanningTime(std::stod(global_parameters.get_planner_parameters()["planning_time"]));
    //nh.getParam("planner_id", _planner_id);
    _my_spinner.reset(new ros::AsyncSpinner(1));
    _my_spinner->start();
}

bool BAXTER_Mover::move_baxter_arm_cb(baxter_mover_utils::move_baxter_arm::Request &req,
                                      baxter_mover_utils::move_baxter_arm::Response &res){

    baxter_helpers_methods::prepare_motion_request(req, res, global_parameters);

    bool plan_success = false, execute_success = false;
    if(!req.collision_detection){
        _clear_octomap->call(global_parameters.get_empty_octomap_request(), global_parameters.get_empty_octomap_response());
        if(_get_motion_plan->call(global_parameters.get_motion_request(), global_parameters.get_motion_response()))
            plan_success = true;
        //if didn't work first time try 10 more times
        else{
            for(int i = 0; i < 10 || !plan_success; i++){
                ROS_WARN("trying to move without checking collision :):):):)");
                _clear_octomap->call(global_parameters.get_empty_octomap_request(), global_parameters.get_empty_octomap_response());
                if(_get_motion_plan->call(global_parameters.get_motion_request(), global_parameters.get_motion_response()))
                    plan_success = true;
            }
        }
    }
    else if(_get_motion_plan->call(global_parameters.get_motion_request(), global_parameters.get_motion_response())){
        plan_success = true;

    }
    else
        plan_success = false;

    if(plan_success){
        global_parameters.get_motion_execute_request().trajectory = global_parameters.get_motion_response().motion_plan_response.trajectory;
        ROS_ERROR_STREAM("size of trajectory to be executed is: "
                         << global_parameters.get_motion_execute_request().trajectory.joint_trajectory.points.size());
        global_parameters.get_motion_execute_request().wait_for_execution = true;
        if(_execute_motion_plan->call(global_parameters.get_motion_execute_request(), global_parameters.get_motion_execute_response()))
            execute_success = true;
    }

    ROS_INFO_STREAM("the planning response error_code is: " << global_parameters.get_motion_response().motion_plan_response.error_code);
    ROS_INFO_STREAM("and planning success is: " << plan_success);
    ROS_INFO_STREAM("and executing success is: " << execute_success);

    return execute_success;
}

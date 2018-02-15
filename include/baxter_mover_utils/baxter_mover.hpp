#ifndef _BAXTER_MOVER_HPP
#define _BAXTER_MOVER_HPP
#include <baxter_mover_utils/helpers_methods.hpp>

namespace baxter_mover {
class BAXTER_Mover {
public:
    typedef std::shared_ptr<BAXTER_Mover> Ptr;
    typedef const std::shared_ptr<BAXTER_Mover> ConstPtr;

    BAXTER_Mover(ros::NodeHandle& nh){
        //nh_.reset(new ros::NodeHandle(nh, "baxter_mover"));
        init(nh);

    }

    void init(ros::NodeHandle& nh);
    bool move_baxter_arm_cb(baxter_mover_utils::move_baxter_arm::Request& req,
                            baxter_mover_utils::move_baxter_arm::Response& res);

    //call back that register baxter left end effector pose and rearrange the orientation in RPY
    void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
        baxter_helpers_methods::locate_eef_pose(l_eef_feedback.pose, global_parameters, "left_gripper");
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                    //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
    }

    //call back that register baxter right end effector pose and rearrange the orientation in RPY
    void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
        baxter_helpers_methods::locate_eef_pose(r_eef_feedback.pose, global_parameters, "right_gripper");
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                    //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
    }

    //call back that register crustcrawler joint states
    void joint_state_Callback(const sensor_msgs::JointState::ConstPtr& joint_state_feedback){
        //ROS_INFO_STREAM("BAXTER MOVER: I am saving joint states, that is good :)");
        if(joint_state_feedback->position.size() > 7)
            global_parameters.set_joint_state(joint_state_feedback);
    }

    void call_service_get_ps(){
        global_parameters.get_ps_request().components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        _get_planning_scene->call(global_parameters.get_ps_request(), global_parameters.get_ps_response());
        if(global_parameters.get_adding_octomap_to_acm()){
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.push_back(true);
        }
        else{
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_names.clear();
            global_parameters.get_ps_response().scene.allowed_collision_matrix.default_entry_values.clear();
        }
    }

    void publish_psm_msg(){
        global_parameters.get_ps_msg().is_diff = true;
        global_parameters.get_ps_msg().allowed_collision_matrix = global_parameters.get_ps_response().scene.allowed_collision_matrix;
        _psm_pub->publish(global_parameters.get_ps_msg());
    }

    void publish_psm_msg(moveit_msgs::PlanningScene planning_scene){
        _psm_pub->publish(planning_scene);
    }

    Data_config global_parameters;
    std::shared_ptr<moveit::planning_interface::MoveGroup> group, secondary_group;

private:
    std::unique_ptr<ros::AsyncSpinner> _my_spinner;
    std::unique_ptr<ros::ServiceServer> _baxter_mover;
    std::unique_ptr<ros::ServiceClient> _get_motion_plan;
    std::unique_ptr<ros::ServiceClient> _execute_motion_plan;
    std::unique_ptr<ros::ServiceClient> _clear_octomap;
    std::unique_ptr<ros::ServiceClient> _get_planning_scene;
    std::unique_ptr<ros::Publisher> _psm_pub;
    std::shared_ptr<moveit::planning_interface::MoveGroup> _group;
    std::unique_ptr<ros::Subscriber> _sub_l_eef_msg, _sub_r_eef_msg;
    std::string _planner_id;
    std::unique_ptr<ros::Subscriber> _sub_joint_state_msg;
    XmlRpc::XmlRpcValue _planner_parameters;

    ros::NodeHandlePtr _nh;
};


}

#endif //_BAXTER_MOVER_HPP

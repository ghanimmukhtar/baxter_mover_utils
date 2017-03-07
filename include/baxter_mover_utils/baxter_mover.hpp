#ifndef _BAXTER_MOVER_HPP
#define _BAXTER_MOVER_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <memory>
#include <boost/bind.hpp>

#include <baxter_mover_utils/parameters.hpp>
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
        baxter_helpers_methods::locate_eef_pose(l_eef_feedback.pose, _global_parameters, "left_gripper");
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                    //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
    }

    //call back that register baxter right end effector pose and rearrange the orientation in RPY
    void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
        baxter_helpers_methods::locate_eef_pose(r_eef_feedback.pose, _global_parameters, "right_gripper");
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                    //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
    }

private:
    Data_config _global_parameters;
    std::unique_ptr<ros::AsyncSpinner> _my_spinner;
    std::unique_ptr<ros::ServiceServer> _baxter_mover;
    std::unique_ptr<ros::ServiceClient> _get_motion_plan;
    std::unique_ptr<ros::ServiceClient> _execute_motion_plan;
    std::unique_ptr<ros::Subscriber> _sub_l_eef_msg, _sub_r_eef_msg;
    std::string _planner_id;
    XmlRpc::XmlRpcValue _planner_parameters;

    ros::NodeHandlePtr nh_;
};


}

#endif //_BAXTER_MOVER_HPP

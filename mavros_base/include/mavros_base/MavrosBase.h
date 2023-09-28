#pragma once

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

class MavrosBase{
    public: 
        MavrosBase();
        bool arm(int n_retry=5);
        bool disarm(int n_retry=5);
        bool set_mode(std::string mode, int n_retry=5);
        void abort_mission(bool immediate=false);
        bool set_rate(int id, int rate);
        bool get_param(std::string param_id, double& param);
        bool set_param(std::string param_id, double param);
        // bool set_param(std::string param_id, int param);

    protected:
        ros::NodeHandle nh; // ROS node handle
        mavros_msgs::State m_current_state; // The current state of the FCU
        geometry_msgs::PoseStamped m_pose; // The current pose of the robot

    private:
        ros::Subscriber m_pose_sub; // Subscriber to the robot pose
        ros::Subscriber m_state_sub; // Subscriber to the FCU state
        ros::ServiceClient m_arming_srv; // Service client for arming/disarming
        ros::ServiceClient m_set_mode_srv; // Service client for setting the mode
        ros::ServiceClient m_cmd_srv; // Service client for sending MAVROS commands
        ros::ServiceClient m_set_rate_srv; // Service client for setting message rates
        ros::ServiceClient m_get_param_srv; // Service client for getting parameters
        ros::ServiceClient m_set_param_srv; // Service client for setting parameters

        static void _pose_cb(
            const geometry_msgs::PoseStamped::ConstPtr& msg, 
            geometry_msgs::PoseStamped& pose
        );
        static void _state_cb(
            const mavros_msgs::State::ConstPtr& msg, mavros_msgs::State& state
        );
        bool _arm_toggle(bool arm);
        bool _kill_motors();
};
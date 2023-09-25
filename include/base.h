#pragma once


#include <ros/ros.h>
#include <mavros_msgs/State.h>

class MavrosBase{
    public: 
        MavrosBase(int argc, char **argv, std::string& node_name);
        bool arm(int n_retry = 5);
        bool disarm(int n_retry = 5);
        bool set_mode(std::string mode, int n_retry = 5);
        bool set_mode(int mode, int n_retry = 5);
        void abort_mission();
        bool set_rate(int id, int rate);
        bool get_param(std::string param_id, double& param);
        bool set_param(std::string param_id, double param);

    protected:
        ros::NodeHandle nh; // ROS node handle
        mavros_msgs::State m_current_state;
};
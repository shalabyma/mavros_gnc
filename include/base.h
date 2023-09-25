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
    
    protected:
        mavros_msgs::State m_current_state;
        ros::Subscriber m_state_sub;
        ros::ServiceClient m_arming_client;
        ros::ServiceClient m_set_mode_client;
        ros::ServiceClient m_cmd_srv;
};
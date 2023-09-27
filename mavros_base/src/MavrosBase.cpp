/**
 * @file base.cpp
 * @brief 
 */

#include "MavrosBase.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

MavrosBase::MavrosBase(int argc, char **argv, std::string& node_name){
    /* ------------------------ Subscribers ------------------------ */
    m_state_sub = nh.subscribe<mavros_msgs::State>
        (
            "mavros/state", 
            10, 
            boost::bind(
                &MavrosBase::_state_cb, boost::placeholders::_1, boost::ref(m_current_state)
            )
        );

    /* ------------------------ Services ------------------------ */
    int service_timeout = 1e3; // timeout in ms
    try{
        ros::service::waitForService("mavros/cmd/arming", service_timeout);
        ros::service::waitForService("mavros/set_mode", service_timeout);
        ros::service::waitForService("mavros/cmd/command", service_timeout);
        ros::service::waitForService("mavros/set_message_interval", service_timeout);
        ros::service::waitForService("mavros/param/get", service_timeout);
        ros::service::waitForService("mavros/param/set", service_timeout);
        ROS_INFO("Connected to basic services!");
    }
    catch(ros::Exception& e){
        ROS_ERROR(
            "Failed to connect to basic services. Ensure MAVROS is running and connected!"
        );
        ros::shutdown();
    }

    m_arming_srv = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    m_set_mode_srv = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    m_cmd_srv = nh.serviceClient<mavros_msgs::CommandLong>
        ("mavros/cmd/command");
    m_set_rate_srv = nh.serviceClient<mavros_msgs::MessageInterval>
        ("mavros/set_message_interval");
    m_get_param_srv = nh.serviceClient<mavros_msgs::ParamGet>
        ("mavros/param/get");
    m_set_param_srv = nh.serviceClient<mavros_msgs::ParamSet>
        ("mavros/param/set");

    // Wait for FCU connection
    while(ros::ok() && !m_current_state.connected){
        ros::spinOnce();
    }
}

// TODO: Replace with a call to a retry utils function
// TODO: Check if already armed
bool MavrosBase::arm(int n_retry){
    for (int i = 0; i < n_retry; i++){
        if (_arm_toggle(true)){
            return true;
        }
        ros::spinOnce();
        sleep(1);
    }
    return false;
}

// TODO: Replace with a call to a retry utils function
// TODO: Check if already disarmed
bool MavrosBase::disarm(int n_retry){
    for (int i = 0; i < n_retry; i++){
        if (_arm_toggle(false)){
            return true;
        }
        ros::spinOnce();
        sleep(1);
    }
    return false;
}

bool MavrosBase::set_mode(std::string mode, int n_retry){
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = mode;

    for (int i = 0; i < n_retry; i++){
        if( m_current_state.mode != mode){
            if( m_set_mode_srv.call(set_mode) &&
                set_mode.response.mode_sent){
                ROS_INFO("%s enabled", mode.c_str());
                return true;
            }
        }
        ros::spinOnce();
        sleep(1);
    }
    return false;
}

void MavrosBase::abort_mission(bool immediately){
    if (immediately){
        ROS_INFO("Aborting mission. Killing motors immediately!");
        _kill_motors();
        return;
    }
    else{
        ROS_INFO("Aborting mission. Executing emergency landing...");
        if (!set_mode("AUTO.LAND", 3)){
            ROS_ERROR("Failed at emergency landing, killing motors immediately!");
            _kill_motors();
        }
    }
}

bool MavrosBase::set_rate(int id, int rate){
    mavros_msgs::MessageInterval msg;
    msg.request.message_id = id;
    msg.request.message_rate = rate;
    if(m_set_rate_srv.call(msg)){
        ROS_INFO("Successfully set rate for MAVLINK MSG ID %d to %d", id, rate);
        return true;
    }else{
        ROS_ERROR("Failed at setting rate for MAVLINK MSG ID %d to %d", id, rate);
        return false;
    }
}

bool MavrosBase::get_param(std::string param_id, double& param){
    mavros_msgs::ParamGet param_get;
    param_get.request.param_id = param_id;
    if(m_get_param_srv.call(param_get)){
        param = param_get.response.value.real;
        ROS_INFO("Successfully got parameter %s: %f", param_id.c_str(), param);
        return true;
    }else{
        ROS_ERROR("Failed at getting parameter %s", param_id.c_str());
        return false;
    }
}

bool MavrosBase::set_param(std::string param_id, double param){
    mavros_msgs::ParamSet param_set;
    param_set.request.param_id = param_id;
    param_set.request.value.real = param;
    if(m_set_param_srv.call(param_set)){
        ROS_INFO("Successfully set parameter %s to %f", param_id.c_str(), param);
        return true;
    }else{
        ROS_ERROR("Failed at setting parameter %s to %f", param_id.c_str(), param);
        return false;
    }
}

void MavrosBase::_state_cb(
    const mavros_msgs::State::ConstPtr& msg, mavros_msgs::State& state
){
    state = *msg;
}

bool MavrosBase::_arm_toggle(bool arm){
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    if(arm_cmd.request.value == true){
        ROS_INFO("Arming...");
    }else{
        ROS_INFO("Disarming...");
    }

    if(m_arming_srv.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Success: %d", arm_cmd.response.success);
        return true;
    }else{
        ROS_ERROR("Failed: %d", arm_cmd.response.success);
        return false;
    }
}

bool MavrosBase::_kill_motors(){
    mavros_msgs::CommandLong cmd;
    cmd.request.broadcast = false;
    cmd.request.command = 400;
    cmd.request.confirmation = 0;
    cmd.request.param1 = 0.0;
    cmd.request.param2 = 21196.0;
    cmd.request.param4 = 0.0;
    cmd.request.param5 = 0.0;
    cmd.request.param6 = 0.0;
    cmd.request.param7 = 0.0;

    if(m_cmd_srv.call(cmd)){
        ROS_INFO("Successfully killed motors");
        return true;
    }else{
        ROS_ERROR("Failed at killing motors");
        return false;
    }
}
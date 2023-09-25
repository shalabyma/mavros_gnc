/**
 * @file base.cpp
 * @brief 
 */

#include "base.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

class MavrosBase{
public:
    MavrosBase(int argc, char **argv, std::string& node_name){
        ros::init(argc, argv, node_name);

        m_state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, _state_cb);
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
    bool arm(int n_retry = 5){
        for (int i = 0; i < n_retry; i++){
            if (_arm_toggle(true)){
                return true;
            }
            sleep(1);
        }
        return false;
    }

    // TODO: Replace with a call to a retry utils function
    bool disarm(int n_retry = 5){
        for (int i = 0; i < n_retry; i++){
            if (_arm_toggle(false)){
                return true;
            }
            sleep(1);
        }
        return false;
    }

    bool set_mode(std::string mode, int n_retry = 5){
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = mode;

        for (int i = 0; i < n_retry; i++){
            if( m_current_state.mode != mode){
                if( m_set_mode_srv.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("%s enabled", mode);
                    return true;
                }
            }
            sleep(1);
        }
        return false;
    }

    void abort_mission(){
        ROS_INFO("Aborting mission. Executing emergency landing...");
        if (!set_mode("AUTO.LAND", 3)){
            ROS_ERROR("Failed at emergency landing, killing motors immediately!");
            _kill_motors();
        }
    }

    bool set_rate(int id, int rate){
        if(m_set_rate_srv.call(id, rate)){
            ROS_INFO("Successfully set rate for MAVLINK MSG ID %d to %d", id, rate);
            return true;
        }else{
            ROS_ERROR("Failed at setting rate for MAVLINK MSG ID %d to %d", id, rate);
            return false;
        }
    }

    bool get_param(std::string param_id, double& param){
        if(m_get_param_srv.call(param_id, param)){
            ROS_INFO("Successfully got parameter %s: %f", param_id.c_str(), param);
            return true;
        }else{
            ROS_ERROR("Failed at getting parameter %s", param_id.c_str());
            return false;
        }
    }

    bool set_param(std::string param_id, double param){
        if(m_set_param_srv.call(param_id, param)){
            ROS_INFO("Successfully set parameter %s to %f", param_id.c_str(), param);
            return true;
        }else{
            ROS_ERROR("Failed at setting parameter %s to %f", param_id.c_str(), param);
            return false;
        }
    }


protected:
    ros::NodeHandle nh; // ROS node handle
    mavros_msgs::State m_current_state; // The current state of the FCU

private:
    ros::Subscriber m_state_sub; // Subscriber to the FCU state
    ros::ServiceClient m_arming_srv; // Service client for arming/disarming
    ros::ServiceClient m_set_mode_srv; // Service client for setting the mode
    ros::ServiceClient m_cmd_srv; // Service client for sending MAVROS commands
    ros::ServiceClient m_set_rate_srv; // Service client for setting message rates
    ros::ServiceClient m_get_param_srv; // Service client for getting parameters
    ros::ServiceClient m_set_param_srv; // Service client for setting parameters

    void _state_cb(const mavros_msgs::State::ConstPtr& msg){
        m_current_state = *msg;
    }

    bool _arm_toggle(bool arm){
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

    bool _kill_motors(){
        mavros_msgs::CommandLongRequest cmd;
        cmd.broadcast = false;
        cmd.command = 400;
        cmd.confirmation = 0;
        cmd.param1 = 0.0;
        cmd.param2 = 21196.0;
        cmd.param4 = 0.0;
        cmd.param5 = 0.0;
        cmd.param6 = 0.0;
        cmd.param7 = 0.0;

        if(m_cmd_srv.call(cmd)){
            ROS_INFO("Successfully killed motors");
            return true;
        }else{
            ROS_ERROR("Failed at killing motors");
            return false;
        }
    }
};
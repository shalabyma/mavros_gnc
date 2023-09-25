/**
 * @file base.cpp
 * @brief 
 */

#include "base.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>

class MavrosBase{
    public:
        MavrosBase(int argc, char **argv, std::string& node_name){
            ros::init(argc, argv, node_name);
            ros::NodeHandle nh;

            m_state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 10, _state_cb);
            m_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
            m_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
            m_cmd_srv = nh.serviceClient<mavros_msgs::CommandLong>
                ("mavros/cmd/command");

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
                    if( m_set_mode_client.call(offb_set_mode) &&
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


    protected:
        mavros_msgs::State m_current_state;
        ros::Subscriber m_state_sub;
        ros::ServiceClient m_arming_client;
        ros::ServiceClient m_set_mode_client;
        ros::ServiceClient m_cmd_srv;

    private:
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

            if(m_arming_client.call(arm_cmd) && arm_cmd.response.success){
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
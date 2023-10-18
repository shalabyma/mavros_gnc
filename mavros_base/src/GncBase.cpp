/**
 * @file base.cpp
 * @brief 
 */

#include "GncBase.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

GncBase::GncBase(){
    
    // Wait for some important topics to be published 
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose"
    );
    ros::topic::waitForMessage<mavros_msgs::State>(
        "mavros/state"
    );

    /* ------------------------ Subscribers ------------------------ */
    // TODO: the pose might could be shared among all instances of GncBase
    //       this can easily be done by having the subscriber as a static
    //       variable. OR by having a static private counter for the 
    //       number of instances and only give the first instance the 
    //       duty of subscribing to the pose topic and updating the pose
    m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 
        10, 
        boost::bind(
            &GncBase::_pose_cb, boost::placeholders::_1 , boost::ref(m_pose)
        )
    );
    m_state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 
        10, 
        boost::bind(
            &GncBase::_state_cb, boost::placeholders::_1, boost::ref(m_current_state)
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

    // TODO: this should be done once and shared among all instances of GncBase
    // Get all existing ROS namespaces
    _get_namespaces();
}

// TODO: Replace with a call to a retry utils function
// TODO: Check if already armed
bool GncBase::arm(int n_retry){
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
bool GncBase::disarm(int n_retry){
    for (int i = 0; i < n_retry; i++){
        if (_arm_toggle(false)){
            return true;
        }
        ros::spinOnce();
        sleep(1);
    }
    return false;
}

bool GncBase::set_mode(std::string mode, int n_retry){
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

void GncBase::abort_mission(bool immediately){
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

bool GncBase::set_rate(int id, int rate){
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

bool GncBase::get_param(std::string param_id, double& param){
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

bool GncBase::get_param(std::string param_id, int& param){
    mavros_msgs::ParamGet param_get;
    param_get.request.param_id = param_id;
    if(m_get_param_srv.call(param_get)){
        param = param_get.response.value.integer;
        ROS_INFO("Successfully got parameter %s: %d", param_id.c_str(), param);
        return true;
    }else{
        ROS_ERROR("Failed at getting parameter %s", param_id.c_str());
        return false;
    }
}

bool GncBase::set_param(std::string param_id, double param){
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

bool GncBase::set_param(std::string param_id, int param){
    mavros_msgs::ParamSet param_set;
    param_set.request.param_id = param_id;
    param_set.request.value.integer = param;
    if(m_set_param_srv.call(param_set)){
        ROS_INFO("Successfully set parameter %s to %d", param_id.c_str(), param);
        return true;
    }else{
        ROS_ERROR("Failed at setting parameter %s to %d", param_id.c_str(), param);
        return false;
    }
}

/* ------------------------ Private methods ------------------------ */
void GncBase::_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg, 
    geometry_msgs::PoseStamped& pose
){
    pose = *msg;
}

void GncBase::_state_cb(
    const mavros_msgs::State::ConstPtr& msg, mavros_msgs::State& state
){
    state = *msg;
}

bool GncBase::_arm_toggle(bool arm){
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

bool GncBase::_kill_motors(){
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
        ROS_INFO("Successfully killed motors.");
        return true;
    }else{
        ROS_ERROR("Failed at killing motors.");
        return false;
    }
}

void GncBase::_get_namespaces(){
    // This checks if there are any prefixes to /mavros topics, and adds them to
    // the list of ROS namespaces. This is useful for multi-robot simulations.
    // IF there are no prefixes, then the list will only contain the empty string.

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++){
        std::string topic = *(&(it->name));
        std::string delimiter = "/";
        size_t pos = 0;
        std::string token;
        std::vector<std::string> tokens;

        topic.erase(0, 1); // Remove the first slash
        while ((pos = topic.find(delimiter)) != std::string::npos) {
            token = topic.substr(0, pos);
            tokens.push_back(token);
            topic.erase(0, pos + delimiter.length());
        }
        tokens.push_back(topic);

        if (tokens.size() >= 2 && tokens[1] == "mavros") {
            std::string ns = tokens[0];
            if (std::find(m_ros_namespaces.begin(), m_ros_namespaces.end(), ns) == m_ros_namespaces.end()){
                ROS_INFO("Found mavros namespace: %s", tokens[0].c_str());
                m_ros_namespaces.push_back(ns);
            }
        }
    }

    ROS_INFO("Found a total of %lu ROS namespaces.", m_ros_namespaces.size());
}
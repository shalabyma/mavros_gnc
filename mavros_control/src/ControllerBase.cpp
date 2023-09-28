/**
 * @file commands.cpp
 * @brief Tools for sending commands to the PX4 Autopilot via MAVROS in Offboard Mode.
 * 
 * #TODO: 
 * 1) Add support for commanding low-level control inputs
 */

#include "ControllerBase.h"
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>

/* ------------------------ Constructor ------------------------ */
ControllerBase::ControllerBase(): MavrosBase(){

    set_rate(HIGHRES_IMU_ID, 250);
    set_rate(DISTANCE_SENSOR_ID, 30);
    set_rate(ATTITUDE_ID, 10);
    set_rate(ATTITUDE_QUATERNION_ID, 10);

    // Wait for some important topics to be published 
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose"
    );

    /* ------------------------ Subscribers ------------------------ */
    m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 
        10, 
        boost::bind(
            &ControllerBase::_pose_cb, boost::placeholders::_1 , boost::ref(m_pose)
        )
    );

    /* ------------------------ Publishers ------------------------ */
    m_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("mavros/setpoint_raw/local", 10);

    /* ------------------------ Services ------------------------ */


    /* ------------------------ Start thread ------------------------ */
    command_vel(0, 0, 0, 0);
    setpoint_thread = boost::thread(&ControllerBase::_stream_setpoints, this);

    // Allow some setpoint commands so OFFBOARD mode is accepted
    ros::topic::waitForMessage<mavros_msgs::PositionTarget>(
        "mavros/setpoint_raw/local"
    );
    sleep(2);
    
    set_mode("OFFBOARD");

}

/* ------------------------ Public methods ------------------------ */
void ControllerBase::command_vel(float vx, float vy, float vz, float yaw_rate){
    m_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    m_setpoint.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW;
    m_setpoint.velocity.x = vx;
    m_setpoint.velocity.y = vy;
    m_setpoint.velocity.z = vz;
    m_setpoint.yaw_rate = yaw_rate;
}

void ControllerBase::command_pos(float x, float y, float z, float yaw){
    m_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    m_setpoint.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    m_setpoint.position.x = x;
    m_setpoint.position.y = y;
    m_setpoint.position.z = z;
    m_setpoint.yaw = yaw;
}

bool ControllerBase::takeoff(float z, time_t timeout){
    arm();
    sleep(2);
    ROS_INFO("Taking off...");
    while (m_current_state.mode != "OFFBOARD"){
        ROS_INFO("Waiting for OFFBOARD mode...");
        ros::spinOnce();
        sleep(1);
    }
    command_pos(
        m_pose.pose.position.x, 
        m_pose.pose.position.y, 
        z, 
        tf::getYaw(m_pose.pose.orientation)
    );
    time_t start_time = time(NULL);
    ros::Rate rate(50.0);
    while (time(NULL) - start_time < timeout){
        if (m_pose.pose.position.z > (z - 0.1)){
            ROS_INFO("Takeoff complete.");
            return true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff failed.");
    return false;
}

bool ControllerBase::land(time_t timeout){
    set_mode("AUTO.LAND");
    ROS_INFO("Landing...");
    time_t start_time = time(NULL);
    while (time(NULL) - start_time < timeout){
        if (m_pose.pose.position.z < 0.05){
            ROS_INFO("Landing complete.");
            return true;
        }
        ros::spinOnce();
    }
    ROS_INFO("Landing failed.");
    abort_mission(true);
    return false;
}

bool ControllerBase::hold_position(){
    ROS_INFO("Holding position...");
    command_pos(
        m_pose.pose.position.x, 
        m_pose.pose.position.y, 
        m_pose.pose.position.z, 
        tf::getYaw(m_pose.pose.orientation)
    );
    return true;
}

/* ------------------------ Private methods ------------------------ */
void ControllerBase::_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg, 
    geometry_msgs::PoseStamped& pose
){
    pose = *msg;
}

void ControllerBase::_stream_setpoints(){
    ros::Rate rate = ros::Rate(50);
    m_setpoint.header = std_msgs::Header();
    m_setpoint.header.frame_id = "base_footprint";
    while(ros::ok()){
        m_setpoint.header.stamp = ros::Time::now();
        m_setpoint_pub.publish(m_setpoint);
        ros::spinOnce();
        rate.sleep();
    }
}
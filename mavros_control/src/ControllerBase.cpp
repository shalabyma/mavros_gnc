/**
 * @file commands.cpp
 * @brief Tools for sending commands to the PX4 Autopilot via MAVROS in Offboard Mode.
 */

#include "ControllerBase.h"
#include <thread>
#include <std_msgs/Header.h>


// Mavlink message IDs
#define HIGHRES_IMU_ID 105
#define DISTANCE_SENSOR_ID 132
#define ATTITUDE_ID 30
#define ATTITUDE_QUATERNION_ID 31

ControllerBase::ControllerBase(int argc, char **argv, std::string node_name)
    : MavrosBase(argc, argv, node_name){

    set_rate(HIGHRES_IMU_ID, 250);
    set_rate(DISTANCE_SENSOR_ID, 30);
    set_rate(ATTITUDE_ID, 10);
    set_rate(ATTITUDE_QUATERNION_ID, 10);

    /* ------------------------ Subscribers ------------------------ */
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 5, boost::bind(&_pose_cb, _1, m_pose));

    /* ------------------------ Publishers ------------------------ */
    m_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("mavros/setpoint_position/local", 10);

    /* ------------------------ Services ------------------------ */


    /* ------------------------ Start thread ------------------------ */
    std::thread t{&ControllerBase::_stream_setpoints, this};

    sleep(2);
    set_mode("OFFBOARD");

    // TODO:TO BE REMOVED
    sleep(5);
    arm();
}

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
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ControllerBase controller(argc, argv);
    ros::spin();
    return 0;
}
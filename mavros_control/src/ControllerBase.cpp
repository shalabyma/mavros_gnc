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

/* ------------------------ Constructor ------------------------ */
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
        ("mavros/setpoint_raw/local", 10);

    /* ------------------------ Services ------------------------ */


    /* ------------------------ Start thread ------------------------ */
    command_pos(0, 0, 2, 0);
    std::thread t{&ControllerBase::_stream_setpoints, this};

    sleep(2);
    set_mode("OFFBOARD");

    // TODO:TO BE REMOVED
    sleep(5);
    arm();
    while (ros::ok()){
        ros::spinOnce();
    }
}

/* ------------------------ Public methods ------------------------ */
void ControllerBase::command_vel(double vx, double vy, double vz, double yaw_rate){
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

void ControllerBase::command_pos(double x, double y, double z, double yaw){
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
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ControllerBase controller(argc, argv);
    ros::spin();
    return 0;
}
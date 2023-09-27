/**
 * @file commands.cpp
 * @brief Tools for sending commands to the PX4 Autopilot via MAVROS in Offboard Mode.
 */

#include "ControllerBase.h"
#include <thread>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>


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
        (
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
    std::thread t{&ControllerBase::_stream_setpoints, this};

    sleep(2);
    set_mode("OFFBOARD");

    // TODO:TO BE REMOVED
    sleep(5);
    takeoff();
    sleep(5);
    command_pos(4, 2, 2, 0);
    sleep(5);
    hold_position();
    sleep(5);
    land();
    // while (ros::ok()){
    //     ros::spinOnce();
    // }
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

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ControllerBase controller(argc, argv);
    ros::spin();
    return 0;
}
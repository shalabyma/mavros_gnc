/**
 * @file commands.cpp
 * @brief Tools for sending commands to the PX4 Autopilot via MAVROS in Offboard Mode.
 */

#include "base.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <thread>
#include <std_msgs/Header.h>


// Mavlink message IDs
#define HIGHRES_IMU_ID 105
#define DISTANCE_SENSOR_ID 132
#define ATTITUDE_ID 30
#define ATTITUDE_QUATERNION_ID 31

class ControllerBase: public MavrosBase{
public:
    ControllerBase(int argc, char **argv, std::string node_name = "controller")
        : MavrosBase(argc, argv, node_name){

        set_rate(HIGHRES_IMU_ID, 250);
        set_rate(DISTANCE_SENSOR_ID, 30);
        set_rate(ATTITUDE_ID, 10);
        set_rate(ATTITUDE_QUATERNION_ID, 10);

        /* ------------------------ Subscribers ------------------------ */
        ros::Subscriber pose_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 5, _pose_cb);

        /* ------------------------ Publishers ------------------------ */
        m_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_position/local", 10);

        /* ------------------------ Services ------------------------ */


        /* ------------------------ Start thread ------------------------ */
        std::thread t{_stream_setpoints};

        sleep(2);
        set_mode("OFFBOARD");

        // TODO:TO BE REMOVED
        sleep(5);
        arm();
    }

protected:
    geometry_msgs::PoseStamped m_pose;
    mavros_msgs::PositionTarget m_setpoint;

private:
    ros::Publisher m_setpoint_pub;

    void _pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        m_pose = *msg;
    }

    void _stream_setpoints(){
        ros::Rate rate = ros::Rate(50);
        m_setpoint.header = std_msgs::Header();
        m_setpoint.header.frame_id = "base_footprint";
        while(ros::ok()){
            m_setpoint.header.stamp = ros::Time::now();
            m_setpoint_pub.publish(m_setpoint);
            rate.sleep();
        }
    }

};

int main(int argc, char **argv){
    ControllerBase controller(argc, argv);
    ros::spin();
    return 0;
}
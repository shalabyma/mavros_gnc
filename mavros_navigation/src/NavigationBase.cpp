#include "NavigationBase.h"

NavigationBase::NavigationBase(): MavrosBase(){

    /* ------------------------ Parameters ------------------------ */
    sleep(15); // Give PX4 some time to boot
    set_param("EKF2_HGT_MODE", 3);
    set_param("EKF2_AID_MASK", 0b000011000);
    set_param("EKF2_EV_DELAY", 0.0);
    set_param("EKF2_EV_POS_X", 0.0);
    set_param("EKF2_EV_POS_Y", 0.0);
    set_param("EKF2_EV_POS_Z", 0.0);
    set_param("EKF2_MAG_TYPE", 5);
    set_param("MAV_ODOM_LP", 1);

    /* ------------------------ Subscribers ------------------------ */

    /* ------------------------ Publishers ------------------------ */
    m_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/vision_pose/pose", 1
    );

    /* ------------------------ Services ------------------------ */
}

NavigationBase::~NavigationBase(){
    // Restore default parameters
    ROS_INFO("Restoring default parameters...");
    set_param("EKF2_HGT_MODE", 1);
    set_param("EKF2_AID_MASK", 1);
    set_param("EKF2_EV_DELAY", 175.0);
    set_param("EKF2_MAG_TYPE", 0);
    set_param("MAV_ODOM_LP", 0);
}

void NavigationBase::publish_pose(geometry_msgs::PoseStamped pose){
    m_pose_pub.publish(pose);
}
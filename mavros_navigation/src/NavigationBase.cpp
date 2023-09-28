#include "NavigationBase.h"

NavigationBase::NavigationBase(): MavrosBase(){

    /* ------------------------ Parameters ------------------------ */
    set_param("EKF2_HGT_MODE", 3);
    set_param("EKF2_AID_MASK", 0b000011000);
    set_param("EKF2_EV_DELAY", 0.0);
    set_param("EKF2_EV_POS_X", 0.0);
    set_param("EKF2_EV_POS_Y", 0.0);
    set_param("EKF2_EV_POS_Z", 0.0);
    set_param("EKF2_MAG_TYPE", 5);
    set_param("EKF2_MAG_TYPE", 5);
    set_param("MAV_ODOM_LP", 1);

    /* ------------------------ Subscribers ------------------------ */

    /* ------------------------ Publishers ------------------------ */
    m_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/vision_pose/pose", 10
    );

    /* ------------------------ Services ------------------------ */
}

void NavigationBase::publish_pose(geometry_msgs::PoseStamped pose){
    m_pose_pub.publish(pose);
}
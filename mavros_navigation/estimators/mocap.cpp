/**
 * @file mocap.cpp
 * @brief 
 */
#include "NavigationBase.h"

geometry_msgs::PoseStamped mocap_pose;
void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    mocap_pose = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mvnav_mocap_node");
    ros::NodeHandle nh;
    std::string who_am_i = ros::this_node::getNamespace();
    if (who_am_i == "/"){
        who_am_i = "/ifo001/"; //#TODO: replace with an enforced parameter if no namespace
    }

    // Initialize navigation object
    NavigationBase nav;

    // Wait for important topics to be available
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "vrpn_client_node" + who_am_i + "pose"
    );

    // Subscribe to the mocap pose
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "vrpn_client_node" + who_am_i + "pose", 10, mocap_pose_cb
    );

    // Set the rate
    ros::Rate rate(50.0);
    while (ros::ok()){
        // Publish the pose
        nav.publish_pose(mocap_pose);

        // Spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
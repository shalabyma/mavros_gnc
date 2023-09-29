/**
 * @file mocap.cpp
 * @brief 
 */
#include "NavigationBase.h"

bool mocap_available = true;

geometry_msgs::PoseStamped mocap_pose;
ros::Time last_pose_time;
void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (
        msg->pose.position.x != 0.0 && 
        msg->pose.position.y != 0.0 && 
        msg->pose.position.z != 0.0
    ){
        mocap_pose = *msg;
        last_pose_time = ros::Time::now();
    }
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
    sleep(1);

    // Set the rate
    ros::Rate rate(50.0);
    while (!ros::isShuttingDown()){
        // Publish the pose
        nav.publish_pose(mocap_pose);

        // Spin and sleep
        ros::spinOnce();
        rate.sleep();

        if (((ros::Time::now() - last_pose_time).toSec() > 1.0) && mocap_available){
            ROS_WARN("Mocap gap! Executing emergency landing!");
            mocap_available = false;
            nav.abort_mission();
        }
        else if (!mocap_available){
            ROS_INFO("Mocap pose regained.");
            mocap_available = true;
        }
    }

    return 0;
}
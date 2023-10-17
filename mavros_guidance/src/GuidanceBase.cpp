#include "GuidanceBase.h"

/* ------------------------ Constructor ------------------------ */
GuidanceBase::GuidanceBase(){
    // Wait for the local position to be published
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose"
    );

    /* ------------------------ Publishers ------------------------ */
    m_waypoint_pub = nh.advertise<mavros_guidance::Waypoint>("waypoint", 3);
    m_waypoint_seq_pub = \
        nh.advertise<mavros_guidance::WaypointSequence>("waypoint_sequence", 1);

    /* ------------------------ Subscribers ------------------------ */
    // TODO: maybe move this to GncBase and have a bool whether or not to subscribe
    // TODO: should we merge this with the pose subscriber in GncBase?
    // Subscribe to the poses of all robots
    for (int i = 0; i < m_ros_namespaces.size(); i++){
        std::string topic = "/" + m_ros_namespaces[i] + "/mavros/local_position/pose";
        m_pose_subscribers.push_back(
            nh.subscribe<geometry_msgs::PoseStamped>(
                topic, 
                5, 
                boost::bind(
                    &GuidanceBase::_all_pose_cb, 
                    boost::placeholders::_1, 
                    boost::ref(m_pose_all), 
                    i
                )
            )
        );
    }

    /* ------------------------------------------------------------- */
    // Record the start time
    ros::Time start_time = ros::Time::now();

    // TODO: if there is a nearby robot, publish a topic that lets the controller know
}

/* ------------------------ Private methods ------------------------ */
void GuidanceBase::_all_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg, 
    std::vector<geometry_msgs::PoseStamped>& pose_all, 
    int i
){
    pose_all[i] = *msg;
}
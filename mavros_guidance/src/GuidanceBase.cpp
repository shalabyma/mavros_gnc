#include "GuidanceBase.h"
#include <boost/thread.hpp>

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
    // TODO: for example, this should be turned off whenever there is only one robot
    // TODO: should we merge this with the pose subscriber in GncBase?
    // Subscribe to the poses of all robots
    for (int i = 0; i < m_ros_namespaces.size(); i++){
        std::string topic = "/" + m_ros_namespaces[i] + "/mavros/local_position/pose";
        m_pose_subscribers.push_back(
            nh.subscribe<geometry_msgs::PoseStamped>(
                topic, 
                5, 
                boost::bind(
                    &GncBase::_pose_cb, 
                    boost::placeholders::_1, 
                    boost::ref(m_pose_all[i])
                )
            )
        );
    }

    /* ------------------------------------------------------------- */
    // Record the start time
    ros::Time start_time = ros::Time::now();

    /* ------------------------ Start thread ------------------------ */
    boost::thread collision_avoidance_thread = \
                boost::thread(&GuidanceBase::_collision_avoidance, this);
}

/* ------------------------ Private methods ------------------------ */
void GuidanceBase::_collision_avoidance(){
    bool found_nearby_robot = false;
    ros::Rate rate(20.0);
    while (!ros::isShuttingDown()){
        found_nearby_robot = false;
        for (int i = 0; i < m_pose_all.size(); i++){
            if (_check_proximity(m_pose, m_pose_all[i])){
                break;
            };
        }
        if (found_nearby_robot){
            m_nearby_robot = true;
        }
        else{
            m_nearby_robot = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool GuidanceBase::_check_proximity(geometry_msgs::PoseStamped pose1, 
                                    geometry_msgs::PoseStamped pose2){
    // make the proximity threshold a user-defined argument
    double proximity_threshold = 1.0;
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);

    // TODO: remove the poses of the current robot so we do not need to check
    //       for the case where the distance is 0.0
    if ((distance < proximity_threshold) && (distance != 0.0)){
        ROS_INFO("Distance computed: %f", distance);
        return true;
    }
    else{
        return false;
    }
}
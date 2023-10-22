#include "GuidanceBase.h"
#include <boost/thread.hpp>

/* ------------------------ Constructor ------------------------ */
GuidanceBase::GuidanceBase(){
    // Wait for the local position to be published
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose"
    );

    /* ------------------------ Publishers ------------------------ */
    m_waypoint_pub = nh.advertise<mavros_guidance::Waypoint>("guidance/waypoint", 3);
    m_waypoint_seq_pub = \
        nh.advertise<mavros_guidance::WaypointSequence>("guidance/waypoint_sequence", 1);
    m_nearby_robot_pub = nh.advertise<std_msgs::Bool>("guidance/nearby_robot", 1, true);

    /* ------------------------ Subscribers ------------------------ */
    // TODO: maybe move this to GncBase and have a bool whether or not to subscribe
    // TODO: for example, this should be turned off whenever there is only one robot
    // TODO: should we merge this with the pose subscriber in GncBase?
    // Subscribe to the poses of all robots
    for (int i = 0; i < m_ros_namespaces.size(); i++){
        std::string topic = "vrpn_client_node/" + m_ros_namespaces[i] + "/pose";
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
    m_nearby_robot.data = false;
    m_nearby_robot_pub.publish(m_nearby_robot);
    while (!ros::isShuttingDown()){
        found_nearby_robot = false;
        for (int i = 0; i < m_pose_all.size(); i++){
            if (_check_proximity(m_pose, m_pose_all[i])){
                found_nearby_robot = true;
                break;
            };
        }

        // TODO: maybe instead of just publishing a boolean we should publish a
        //       custom message with the ID of the nearby robot and the distance
        // Publish the nearby robot boolean
        if (found_nearby_robot && !m_nearby_robot.data){
            m_nearby_robot.data = true;
            m_nearby_robot_pub.publish(m_nearby_robot);
        }
        else if (!found_nearby_robot && m_nearby_robot.data){
            m_nearby_robot.data = false;
            m_nearby_robot_pub.publish(m_nearby_robot);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

bool GuidanceBase::_check_proximity(
    const geometry_msgs::PoseStamped& pose1, 
    const geometry_msgs::PoseStamped& pose2
){
    // make the proximity threshold a user-defined argument
    double proximity_threshold = 1;
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);

    // TODO: remove the poses of the current robot so we do not need to check
    //       for the case where the distance is <0.05
    if ((distance < proximity_threshold) && (distance > 0.05)){
        return true;
    }
    else{
        return false;
    }
}
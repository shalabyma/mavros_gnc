#include "mavros_guidance/Waypoint.h"
#include "mavros_guidance/WaypointSequence.h"
#include "GncBase.h"
#include <std_msgs/Bool.h>

// TODO: maybe add a toggle for collision avoidance
class GuidanceBase: public GncBase{
public:
    GuidanceBase();

protected:
    ros::Publisher m_waypoint_pub; // Publishes a single waypoint
    ros::Publisher m_waypoint_seq_pub; // Publishes a sequence of waypoints
    std_msgs::Bool m_nearby_robot; // True if another robot is nearby

    // Poses of all robots
    std::vector<geometry_msgs::PoseStamped> m_pose_all = \
        std::vector<geometry_msgs::PoseStamped>(m_ros_namespaces.size());

private:
    // Subscribers to poses of all robots
    std::vector<ros::Subscriber> m_pose_subscribers;

    // Publisher for the nearby robot boolean
    ros::Publisher m_nearby_robot_pub;

    void _collision_avoidance();
    static bool _check_proximity(geometry_msgs::PoseStamped pose1, 
                                 geometry_msgs::PoseStamped pose2);
};
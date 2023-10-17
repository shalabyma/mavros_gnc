#include "mavros_guidance/Waypoint.h"
#include "mavros_guidance/WaypointSequence.h"
#include "GncBase.h"

// TODO: maybe add a toggle for collision avoidance
class GuidanceBase: public GncBase{
public:
    GuidanceBase();

protected:
    ros::Publisher m_waypoint_pub; // Publishes a single waypoint
    ros::Publisher m_waypoint_seq_pub; // Publishes a sequence of waypoints
    bool m_nearby_robot = false; // True if another robot is nearby

    // Poses of all robots
    std::vector<geometry_msgs::PoseStamped> m_pose_all;

private:
    // Subscribers to poses of all robots
    std::vector<ros::Subscriber> m_pose_subscribers;
    
    static void _all_pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg, 
        std::vector<geometry_msgs::PoseStamped>& pose_all, 
        int i
    );

    void _collision_avoidance();
    void _check_proximity();
    void _compute_distance();
};
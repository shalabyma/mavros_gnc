/**
 * @file NavigationBase.h
 * @brief 
 * 
 * #TODO:
 * 1) Should add subscribers and "get" methods for common mavros sensor topics 
 */
#include "MavrosBase.h"
#include <geometry_msgs/PoseStamped.h>

class NavigationBase: public MavrosBase{
public:
    NavigationBase();
    ~NavigationBase();
    void publish_pose(geometry_msgs::PoseStamped pose);

protected:

private:
    ros::Publisher m_pose_pub; // Publisher for the robot vision pose

};
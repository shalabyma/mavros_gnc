#include "MavrosBase.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <boost/thread.hpp>

// Mavlink message IDs
#define HIGHRES_IMU_ID 105
#define DISTANCE_SENSOR_ID 132
#define ATTITUDE_ID 30
#define ATTITUDE_QUATERNION_ID 31

class ControllerBase: public MavrosBase{
public:
    ControllerBase();

    // TODO: is there need for command_yaw_rate and command_yaw separately?
    void command_vel(float vx, float vy, float vz, float yaw_rate=0);
    void command_pos(float x, float y, float z, float yaw=0);

    bool takeoff(float z=1, time_t timeout=20);
    bool land(time_t timeout=20);
    bool hold_position();

protected:
    geometry_msgs::PoseStamped m_pose;
    mavros_msgs::PositionTarget m_setpoint;

private:
    ros::Subscriber m_pose_sub;
    ros::Publisher m_setpoint_pub;
    boost::thread setpoint_thread;

    static void _pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg, 
        geometry_msgs::PoseStamped& pose
    );

    void _stream_setpoints();
};

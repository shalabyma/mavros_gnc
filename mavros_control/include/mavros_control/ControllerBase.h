#include "GncBase.h"
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>

// Mavlink message IDs
#define HIGHRES_IMU_ID 105
#define DISTANCE_SENSOR_ID 132
#define ATTITUDE_ID 30
#define ATTITUDE_QUATERNION_ID 31

class ControllerBase: public GncBase{
public:
    ControllerBase();

    // TODO: is there need for command_yaw_rate and command_yaw separately?
    void command_vel(float vx, float vy, float vz, float yaw_rate=0);
    void command_pos(float x, float y, float z, float yaw=0);

    bool takeoff(float z=1, time_t timeout=20);
    bool land(time_t timeout=20);
    bool hold_position();

protected:
    mavros_msgs::PositionTarget m_setpoint;

private:
    ros::Publisher m_setpoint_pub;
    ros::Subscriber m_collision_avoidance_sub;

    bool m_nearby_robot = false;

    void _stream_setpoints();
    static void _collision_avoidance_cb(
        const std_msgs::Bool::ConstPtr& msg, 
        bool& nearby_robot
    );
};

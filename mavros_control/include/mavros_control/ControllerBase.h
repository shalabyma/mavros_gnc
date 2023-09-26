#include <MavrosBase.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

// Mavlink message IDs
#define HIGHRES_IMU_ID 105
#define DISTANCE_SENSOR_ID 132
#define ATTITUDE_ID 30
#define ATTITUDE_QUATERNION_ID 31

class ControllerBase: public MavrosBase{
public:
    ControllerBase(int argc, char **argv, std::string node_name = "controller");

protected:
    geometry_msgs::PoseStamped m_pose;
    mavros_msgs::PositionTarget m_setpoint;

private:
    ros::Publisher m_setpoint_pub;

    void _pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg, 
        geometry_msgs::PoseStamped& m_pose
    ){}

    void _stream_setpoints();
};

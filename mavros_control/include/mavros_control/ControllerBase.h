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

    // TODO: is there need for command_yaw_rate and command_yaw separately?
    void command_vel(double vx=0, double vy=0, double vz=0, double yaw_rate=0);
    void command_pos(double x=0, double y=0, double z=0, double yaw=0);

protected:
    geometry_msgs::PoseStamped m_pose;
    mavros_msgs::PositionTarget m_setpoint;

private:
    ros::Publisher m_setpoint_pub;

    static void _pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg, 
        geometry_msgs::PoseStamped& pose
    );

    void _stream_setpoints();
};

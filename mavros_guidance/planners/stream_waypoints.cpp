/* 
TODO: we should have 3 cpp files, 
    1) one that publishes waypoints one at a time
    2) one that publishes a sequence of waypoints all at once
    3) and then in a utils folder a cpp file that can be used to convert csv to array or something
*/

#include "GuidanceBase.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    GuidanceBase planner;
    // void _stream_waypoints(mavros_guidance::WaypointSequence waypoint_seq);

    ROS_INFO("Spinning now...");
    ros::Rate rate(1.0);
    while (!ros::isShuttingDown()){        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
/**
 * @file basic_flight.cpp
 * @brief An example of showing basic flight commands.
 */

#include "../include/mavros_control/ControllerBase.h"
#include <iostream>

int main(int argc, char **argv){
    // Initialize the node
    ros::init(argc, argv, "controller");

    // Initialize the controller
    ControllerBase controller;

    // Takeoff
    if(!controller.takeoff())
        return 0;
    sleep(5);

    // Move to a position
    controller.command_pos(4, 2, 2, 0);
    sleep(5);

    // Hold a position
    if (!controller.hold_position())
        return 0;
    sleep(5);

    // Land
    controller.land();
    
    return 0;
}
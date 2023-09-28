/**
 * @file basic_flight.cpp
 * @brief An example of showing basic flight commands.
 */

#include "../include/mavros_control/ControllerBase.h"
#include <iostream>

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ControllerBase controller;
    controller.takeoff();
    sleep(5);
    controller.command_pos(4, 2, 2, 0);
    sleep(5);
    controller.hold_position();
    sleep(5);
    controller.land();
    return 0;
}
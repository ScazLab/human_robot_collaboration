#include <stdio.h>

#include <ros/ros.h>
#include "robot_interface/artag_ctrl.h"
#include "robot_interface/hold_ctrl.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "action_provider");

    ARTagCtrl  left_ctrl("action_provider","left");
    HoldCtrl  right_ctrl("action_provider","right");

    printf("\n");
    ROS_INFO("READY! Waiting for service messages..\n");

    ros::spin();
    return 0;
}

#include <stdio.h>

#include <ros/ros.h>
#include "artag_hold_ctrl.h"
#include "../modular_furniture/part_picker.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "action_provider");
    ros::NodeHandle _n("action_provider");

    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    printf("\n");
    ARTagHoldCtrl right_ctrl("action_provider", "right", use_robot);
    printf("\n");
    PartPicker    left_ctrl ("action_provider",  "left", use_robot);
    printf("\n");
    ROS_INFO("READY! Waiting for service messages..\n");

    ros::spin();
    return 0;
}


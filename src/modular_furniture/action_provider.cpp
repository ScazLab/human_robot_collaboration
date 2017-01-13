#include <stdio.h>

#include <ros/ros.h>
#include "screwdriver_picker.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "action_provider");
    ros::NodeHandle _n("action_provider");

    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    cv::Mat size(1, 2, CV_32FC1);
    size.at<float>(0,0) = 0.080;
    size.at<float>(0,1) = 0.020;

    // printf("\n");
    ScrewDriverPicker test("screwdriver_picker", size);
    // printf("\n");
    // HoldCtrl  right_ctrl("action_provider","right", !use_robot);
    printf("\n");
    ROS_INFO("READY! Waiting for service messages..\n");

    ros::spin();
    return 0;
}


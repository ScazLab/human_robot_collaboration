#include <stdio.h>

#include <ros/ros.h>
#include "robot_perception/cartesian_estimator_hsv.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "hsv_detector");
    ros::NodeHandle _n("hsv_detector");

    CartesianEstimatorHSV ce_hsv("hsv_detector");
    ROS_INFO("READY!\n");

    ros::spin();
    return 0;
}


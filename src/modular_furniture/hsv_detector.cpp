#include <stdio.h>

#include <ros/ros.h>
#include "robot_perception/cartesian_estimator_hsv.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "hsv_detector");
    ros::NodeHandle _n("hsv_detector");

    cv::Mat sizes(2, 2, CV_32FC1);
    // screwdriver
    sizes.at<float>(0,0) = 0.080;
    sizes.at<float>(0,1) = 0.020;
    // blue cube
    sizes.at<float>(1,0) = 0.111;
    sizes.at<float>(1,1) = 0.052;

    hsvColorRange   blue(colorRange( 60, 130), colorRange(90, 256), colorRange( 10,256));
    hsvColorRange yellow(colorRange( 10,  60), colorRange(50, 116), colorRange(120,146));
    hsvColorRange    red(colorRange(160,  10), colorRange(70, 166), colorRange( 10, 66));

    std::vector<hsvColorRange> colors;
    colors.push_back(red);
    colors.push_back(blue);

    std::vector<std::string> names;
    names.push_back("screwdriver");
    names.push_back("blue_box");

    printf("\n");
    CartesianEstimatorHSV ce_hsv("hsv_detector", names, sizes, colors);
    ROS_INFO("READY!\n");

    ros::spin();
    return 0;
}


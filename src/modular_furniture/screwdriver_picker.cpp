#include <stdio.h>

#include <ros/ros.h>
#include "screwdriver_picker.h"

using namespace std;

ScrewDriverPicker::ScrewDriverPicker(std::string name, std::vector<double> _obj_size) : CartesianEstimator(name, _obj_size)
{

}

void ScrewDriverPicker::detectObject(const cv::Mat& _in, cv::Mat& _out)
{
    ROS_INFO("[detectObject]");

    cv::Mat img_hsv;
    cv::cvtColor(_in, img_hsv, CV_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat img_thres;
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    int h_min = 60, h_max = 130;
    int s_min = 90, s_max = 256;
    int v_min = 10, v_max = 256;

    inRange(img_hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), img_thres);

    // Some morphological operations to remove noise and clean up the image
    for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());
    for (int i = 0; i < 4; ++i) dilate(img_thres, img_thres, cv::Mat());
    for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());

    cv::imshow("Test", img_thres);
    cv::waitKey(3);

    // Find contours
    cv::findContours(img_thres, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    if (contours.size() ==1)
    {
        setSegmentedObject(minAreaRect(cv::Mat(contours[0])));
    }

    cv::RNG rng(ros::Time::now().toNSec());
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    // contour
    drawContours( img_thres, contours, 0, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
    // rotated rectangle
    cv::Point2f rect_points[4];
    getSegmentedObject().points( rect_points );

    for( int j = 0; j < 4; j++ )
    {
        cv::line( _out, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    calculateCartesianPosition();
    draw3dAxis(_out);

    return;
}

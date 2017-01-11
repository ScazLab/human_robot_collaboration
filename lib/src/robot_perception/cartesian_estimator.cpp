#include "robot_perception/cartesian_estimator.h"

using namespace std;

CartesianEstimator::CartesianEstimator(string name) : ROSThreadImage(name)
{
    sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", _n);//, 10.0);
    cam_param = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, true);    // For now, we'll assume images that are always rectified
}

void CartesianEstimator::calculateCartesianPosition()
{
    return;
}

CartesianEstimator::~CartesianEstimator()
{

}

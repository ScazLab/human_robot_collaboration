#include "robot_perception/cartesian_estimator.h"

using namespace std;

CartesianEstimator::CartesianEstimator(string name) : ROSThreadImage(name)
{
    sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", _n);//, 10.0);
    cam_param = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, true);    // For now, we'll assume images that are always rectified

    Rvec.create(3,1,CV_32FC1);
    Tvec.create(3,1,CV_32FC1);
    for (int i=0;i<3;i++)
    {
        Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
    }
}

void CartesianEstimator::calculateCartesianPosition()
{
    cv::Mat ImagePoints(4,2,CV_32FC1);
    //Set image points from the rotated rectangle that defines the segmented object
    cv::Point2f obj_segm_pts[4];
    obj_segm.points(obj_segm_pts);

    for (int i=0; i<4; i++)
    {
        ImagePoints.at<float>(i,0)=obj_segm_pts[i].x;
        ImagePoints.at<float>(i,1)=obj_segm_pts[i].y;
    }

    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImagePoints, camMatrix, cv::Mat(), raux, taux);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);
    return;
}

CartesianEstimator::~CartesianEstimator()
{

}

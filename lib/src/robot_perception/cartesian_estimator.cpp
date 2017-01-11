#include "robot_perception/cartesian_estimator.h"

using namespace std;

CartesianEstimator::CartesianEstimator(string name, std::vector<double> _obj_size) : ROSThreadImage(name)
{
    sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", _n);//, 10.0);
    cam_param = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, true);    // For now, we'll assume images that are always rectified

    Rvec.create(3,1,CV_32FC1);
    Tvec.create(3,1,CV_32FC1);
    for (int i=0;i<3;i++)
    {
        Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
    }

    // Let's put the longer size first
    if (_obj_size.size() != 2)
    {
        ROS_ERROR("Size of object should be composed of two elements!");
    }
    else
    {
        if (_obj_size[0] > _obj_size[1])
        {
            obj_size.push_back(_obj_size[0]);
            obj_size.push_back(_obj_size[1]);
        }
        else
        {
            obj_size.push_back(_obj_size[1]);
            obj_size.push_back(_obj_size[0]);
        }
    }
}

bool CartesianEstimator::calculateCartesianPosition()
{
    cv::Mat ObjPoints(4,3,CV_32FC1);
    ObjPoints.at<float>(1,0)=-obj_size[0];
    ObjPoints.at<float>(1,1)=+obj_size[1];
    ObjPoints.at<float>(1,2)=0;
    ObjPoints.at<float>(2,0)=+obj_size[0];
    ObjPoints.at<float>(2,1)=+obj_size[1];
    ObjPoints.at<float>(2,2)=0;
    ObjPoints.at<float>(3,0)=+obj_size[0];
    ObjPoints.at<float>(3,1)=-obj_size[1];
    ObjPoints.at<float>(3,2)=0;
    ObjPoints.at<float>(0,0)=-obj_size[0];
    ObjPoints.at<float>(0,1)=-obj_size[1];
    ObjPoints.at<float>(0,2)=0;

    cv::Mat ImgPoints(4,2,CV_32FC1);
    //Set image points from the rotated rectangle that defines the segmented object
    cv::Point2f obj_segm_pts[4];
    obj_segm.points(obj_segm_pts);

    for (int i=0; i<4; i++)
    {
        ImgPoints.at<float>(i,0)=obj_segm_pts[i].x;
        ImgPoints.at<float>(i,1)=obj_segm_pts[i].y;
    }

    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImgPoints, cam_param.CameraMatrix, cv::Mat(), raux, taux);
    raux.convertTo(Rvec, CV_32F);
    taux.convertTo(Tvec, CV_32F);
    return true;
}

CartesianEstimator::~CartesianEstimator()
{

}

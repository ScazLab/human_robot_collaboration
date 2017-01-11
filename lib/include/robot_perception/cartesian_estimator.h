#ifndef __CARTESIAN_ESTIMATOR__
#define __CARTESIAN_ESTIMATOR__

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>

#include <aruco/cameraparameters.h>
#include <aruco_ros/aruco_ros_utils.h>

#include "robot_utils/ros_thread_image.h"

class CartesianEstimator : public ROSThreadImage
{
private:
    cv::RotatedRect          obj_segm;
    aruco::CameraParameters cam_param;

    //matrices of rotation and translation respect to the camera
    cv::Mat Rvec,Tvec;

protected:
    void calculateCartesianPosition();

public:
    CartesianEstimator(std::string name);
    ~CartesianEstimator();

};

#endif

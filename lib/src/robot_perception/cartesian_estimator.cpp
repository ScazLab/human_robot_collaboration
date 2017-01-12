#include "robot_perception/cartesian_estimator.h"

using namespace std;

CartesianEstimator::CartesianEstimator(string name, std::vector<double> _obj_size) :
                                                              ROSThreadImage(name)
{
    img_pub = _img_trp.advertise("/"+getName()+"/result", 1);

    _n.param<std::string>("/"+getName()+"/reference_frame", reference_frame_, "");
    _n.param<std::string>("/"+getName()+   "/camera_frame",    camera_frame_, "");

    ROS_ASSERT_MSG(not camera_frame_.empty(), "Camera frame is empty!");

    if(reference_frame_.empty()) reference_frame_ = camera_frame_;

    Rvec.create(3,1,CV_32FC1);
    Tvec.create(3,1,CV_32FC1);
    for (int i=0;i<3;i++)
    {
        Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
    }

    ROS_ASSERT_MSG(_obj_size.size() == 2, "Size of object should be composed"
                   " of two elements, %lu found instead", _obj_size.size());

    // Let's put the longer size first
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

    sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
                                                           ("/"+getName()+"/camera_info", _n);//, 10.0);
    // For now, we'll assume images that are always rectified
    cam_param = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, true);
}

void CartesianEstimator::InternalThreadEntry()
{
    while(ros::ok())
    {
        cv::Mat img_in;
        cv::Mat img_out;
        if (!_img_empty)
        {
            pthread_mutex_lock(&_mutex_img);
            img_in=_curr_img;
            pthread_mutex_unlock(&_mutex_img);
            img_out = img_in.clone();

            detectObject(img_in, img_out);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "bgr8", img_out).toImageMsg();
            img_pub.publish(msg);
        }
        r.sleep();
    }
}

bool CartesianEstimator::calcPoseCameraFrame()
{
    // Let's be sure that the width of the RotatedRect is the longest,
    // in order to ensure consistency in the computation of the orientation
    if (obj_segm.size.height > obj_segm.size.width)
    {
        float tmp = obj_segm.size.height;

        obj_segm.size.height = obj_segm.size.width;
        obj_segm.size.width  = tmp;
    }

    // Set image points from the rotated rectangle that defines the segmented object
    cv::Mat ImgPoints(4,2,CV_32FC1);

    cv::Point2f obj_segm_pts[4];
    obj_segm.points(obj_segm_pts);

    for (int i=0; i<4; i++)
    {
        ImgPoints.at<float>(i,0)=obj_segm_pts[i].x;
        ImgPoints.at<float>(i,1)=obj_segm_pts[i].y;
    }

    // Matrix representing the points relative to the objects.
    // The convention used is to have 0 in the bottom left,
    // with the others organized in a clockwise manner
    cv::Mat ObjPoints(4,3,CV_32FC1);
    ObjPoints.at<float>(0,0)=-obj_size[0];
    ObjPoints.at<float>(0,1)=-obj_size[1];
    ObjPoints.at<float>(0,2)=0;
    ObjPoints.at<float>(1,0)=-obj_size[0];
    ObjPoints.at<float>(1,1)=+obj_size[1];
    ObjPoints.at<float>(1,2)=0;
    ObjPoints.at<float>(2,0)=+obj_size[0];
    ObjPoints.at<float>(2,1)=+obj_size[1];
    ObjPoints.at<float>(2,2)=0;
    ObjPoints.at<float>(3,0)=+obj_size[0];
    ObjPoints.at<float>(3,1)=-obj_size[1];
    ObjPoints.at<float>(3,2)=0;

    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImgPoints, cam_param.CameraMatrix, cv::Mat(), raux, taux);
    raux.convertTo(Rvec, CV_32F);
    taux.convertTo(Tvec, CV_32F);
    return true;
}

bool CartesianEstimator::cameraToRootFramePose()
{
    // Get the current transform from the camera frame to output ref frame
    tf::StampedTransform cameraToReference;
    cameraToReference.setIdentity();

    if ( reference_frame_ != camera_frame_ )
    {
        getTransform(reference_frame_, camera_frame_, cameraToReference);
    }

    // Now find the transform the detected object

    tf::Transform transform = object2Tf();
    transform = static_cast<tf::Transform>(cameraToReference) * transform;
    tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          reference_frame_, getName()+"/detected_obj"));

    return true;
}

bool CartesianEstimator::getTransform(const std::string& refFrame,
                                      const std::string& childFrame,
                                      tf::StampedTransform& transform)
{
    std::string errMsg;

    if(!tfListener_.waitForTransform(refFrame, childFrame, ros::Time(0),
                                     ros::Duration(0.5), ros::Duration(0.01), &errMsg))
    {
        ROS_ERROR("Unable to get pose from TF: %s", errMsg.c_str());
        return false;
    }
    else
    {
        try
        {
            tfListener_.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
        }
        catch ( const tf::TransformException& e)
        {
            ROS_ERROR("Error in lookupTransform of %s in %s ",
                        childFrame.c_str(), refFrame.c_str());
            return false;
        }
    }
    return true;
}

tf::Transform CartesianEstimator::object2Tf()
{
    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(Rvec, rot);
    cv::Mat tran = Tvec;

    cv::Mat rotate_to_ros(3, 3, CV_32FC1);
    // -1 0 0
    // 0 0 1
    // 0 1 0
    rotate_to_ros.at<float>(0,0) = -1.0;
    rotate_to_ros.at<float>(0,1) = 0.0;
    rotate_to_ros.at<float>(0,2) = 0.0;
    rotate_to_ros.at<float>(1,0) = 0.0;
    rotate_to_ros.at<float>(1,1) = 0.0;
    rotate_to_ros.at<float>(1,2) = 1.0;
    rotate_to_ros.at<float>(2,0) = 0.0;
    rotate_to_ros.at<float>(2,1) = 1.0;
    rotate_to_ros.at<float>(2,2) = 0.0;
    rot = rot*rotate_to_ros.t();

    tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                         rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                         rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    tf::Vector3 tf_orig(tran.at<float>(0,0), tran.at<float>(1,0), tran.at<float>(2,0));

    return tf::Transform(tf_rot, tf_orig);
}


void CartesianEstimator::draw3dAxis(cv::Mat &_in)
{
    int fFace  = cv::FONT_HERSHEY_SIMPLEX;
    float size=0.2;

    cv::Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0)=0;
    objectPoints.at<float>(0,1)=0;
    objectPoints.at<float>(0,2)=0;
    objectPoints.at<float>(1,0)=size;
    objectPoints.at<float>(1,1)=0;
    objectPoints.at<float>(1,2)=0;
    objectPoints.at<float>(2,0)=0;
    objectPoints.at<float>(2,1)=size;
    objectPoints.at<float>(2,2)=0;
    objectPoints.at<float>(3,0)=0;
    objectPoints.at<float>(3,1)=0;
    objectPoints.at<float>(3,2)=size;

    vector<cv::Point2f> imagePoints;
    cv::projectPoints( objectPoints, Rvec, Tvec, cam_param.CameraMatrix,
                                     cam_param.Distorsion, imagePoints);
    //draw lines of different colours
    cv::line(_in,imagePoints[0],imagePoints[1],cv::Scalar(255,0,0,255),1,CV_AA);
    cv::line(_in,imagePoints[0],imagePoints[2],cv::Scalar(0,255,0,255),1,CV_AA);
    cv::line(_in,imagePoints[0],imagePoints[3],cv::Scalar(0,0,255,255),1,CV_AA);
    cv::putText(_in,"x", imagePoints[1], fFace, 0.6, cv::Scalar(255,0,0,255),2);
    cv::putText(_in,"y", imagePoints[2], fFace, 0.6, cv::Scalar(0,255,0,255),2);
    cv::putText(_in,"z", imagePoints[3], fFace, 0.6, cv::Scalar(0,0,255,255),2);
}

CartesianEstimator::~CartesianEstimator()
{

}

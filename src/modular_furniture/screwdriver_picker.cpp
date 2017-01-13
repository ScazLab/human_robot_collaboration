#include "screwdriver_picker.h"

using namespace std;

ScrewDriverPicker::ScrewDriverPicker(std::string _name, cv::Mat _objs_size) : CartesianEstimator(_name, _objs_size)
{

}

bool ScrewDriverPicker::detectObject(const cv::Mat& _in, cv::Mat& _out)
{
    // ROS_INFO_THROTTLE(1, "[detectObject]");

    // cv::Mat img_hsv;
    // cv::cvtColor(_in, img_hsv, CV_BGR2HSV); //Convert the captured frame from BGR to HSV


    // vector<vector<cv::Point> > contours;
    // vector<cv::Vec4i> hierarchy;

    // hsvColorRange   blue(colorRange( 60, 130), colorRange(90, 256), colorRange( 10,256));
    // hsvColorRange yellow(colorRange( 10,  60), colorRange(50, 116), colorRange(120,146));
    // hsvColorRange    red(colorRange(160,  10), colorRange(70, 166), colorRange( 10, 66));

    // cv::Mat img_thres = hsvThreshold(img_hsv, red);

    // // Some morphological operations to remove noise and clean up the image
    // for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());
    // for (int i = 0; i < 4; ++i) dilate(img_thres, img_thres, cv::Mat());
    // for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());

    // cv::imshow("Test", img_thres);
    // cv::waitKey(3);

    // // Find contours
    // cv::findContours(img_thres, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // if (contours.size() == 1)
    // {
    //     setSegmentedObject(minAreaRect(cv::Mat(contours[0])));
    // }
    // else
    // {
    //     return false;
    // }

    // cv::Scalar color = cv::Scalar::all(255);

    // cv::Point2f rect_points[4];
    // getSegmentedObject().points(rect_points);

    // for( int j = 0; j < 4; j++ )
    // {
    //     cv::line   ( _out, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    //     cv::putText(_out, intToString(j), rect_points[j], cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar::all(255), 3, CV_AA);
    // }

    // calcPoseCameraFrame();
    // cameraToRootFramePose();
    // draw3dAxis(_out);

    return true;
}

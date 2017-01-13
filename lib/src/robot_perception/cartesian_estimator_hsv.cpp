#include "robot_perception/cartesian_estimator_hsv.h"

using namespace std;

SegmentedObjHSV::SegmentedObjHSV(std::vector<double> _size, hsvColorRange _col) :
                                 SegmentedObj(_size), col(_col)
{

}

bool SegmentedObjHSV::detectObject(const cv::Mat& _in, cv::Mat& _out)
{
    cv::Mat out_thres(_in.rows, _in.cols, CV_32FC1, 0.0);
    return detectObject(_in, _out, out_thres);
}

bool SegmentedObjHSV::detectObject(const cv::Mat& _in, cv::Mat& _out, cv::Mat& _out_thres)
{
    cv::Mat img_hsv;
    cv::cvtColor(_in, img_hsv, CV_BGR2HSV); //Convert the captured frame from BGR to HSV

    Contours contours;
    vector<cv::Vec4i> hierarchy;

    cv::Mat img_thres = hsvThreshold(img_hsv, col);

    // Some morphological operations to remove noise and clean up the image
    for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());
    for (int i = 0; i < 4; ++i) dilate(img_thres, img_thres, cv::Mat());
    for (int i = 0; i < 2; ++i) erode(img_thres, img_thres, cv::Mat());

    cv::bitwise_or(_out_thres, img_thres.clone(), _out_thres);

    // Find contours
    cv::findContours(img_thres, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    if (contours.size() == 0) return false;

    // At this stage, there should be only a contour in the vector of contours.
    // If this is not the case, let's pick the contour with the biggest area
    int largest_area=0;
    int largest_contour_idx=-1;

    for( size_t i = 0; i< contours.size(); i++ )
    {
        double a = contourArea( contours[i],false);  //  Find the area of contour

        if( a > largest_area )
        {
            largest_area = a;
            largest_contour_idx = i;                //Store the index of largest contour
        }
    }

    rect = minAreaRect(cv::Mat(contours[largest_contour_idx]));

    cv::Scalar color = cv::Scalar::all(255);

    cv::Point2f rect_points[4];
    rect.points(rect_points);

    for( int j = 0; j < 4; j++ )
    {
        cv::line   (_out, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        cv::putText(_out, intToString(j), rect_points[j],
                     cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar::all(255), 3, CV_AA);
    }

    return true;
}

SegmentedObjHSV::~SegmentedObjHSV()
{

}

CartesianEstimatorHSV::CartesianEstimatorHSV(std::string _name, cv::Mat _objs_size,
                                             std::vector<hsvColorRange> _objs_col) :
                                             CartesianEstimator(_name)
{
    ROS_ASSERT_MSG(_objs_size.cols == 2, "Objects' sizes should have two columns. "
                   "%i found instead", _objs_size.cols);

    objsFromMat(_objs_size, _objs_col);
}

bool CartesianEstimatorHSV::addObject(double _h, double _w, hsvColorRange _hsv)
{
    std::vector<double> size;

    // Let's put the longer size first
    if (_h > _w)
    {
        size.push_back(_h);
        size.push_back(_w);
    }
    else
    {
        size.push_back(_w);
        size.push_back(_h);
    }

    objs.push_back(new SegmentedObjHSV(size, _hsv));

    return true;
}

bool CartesianEstimatorHSV::objsFromMat(cv::Mat _o, std::vector<hsvColorRange> _hsvs)
{
    if (_o.rows != int(_hsvs.size()))
    {
        ROS_ERROR("Rows of the matrix should be equal to the size of the hsv color vector!");
        return false;
    }
    clearObjs();

    bool res = true;

    for (int i = 0; i < _o.rows; ++i)
    {
        res = res && addObject(_o.at<float>(i, 0), _o.at<float>(i, 1), _hsvs[i]);
    }

    return res;
}

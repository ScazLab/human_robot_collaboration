#include "robot_perception/cartesian_estimator_hsv.h"

using namespace std;

/************************************************************************************/
/*                               SEGMENTED OBJECT HSV                               */
/************************************************************************************/
SegmentedObjHSV::SegmentedObjHSV(vector<double> _size, hsvColorRange _col) :
                                 SegmentedObj(_size), col(_col)
{

}

SegmentedObjHSV::SegmentedObjHSV(string _name, vector<double> _size, int _area_thres,
                                 hsvColorRange _col) :
                                 SegmentedObj(_name, _size, _area_thres), col(_col)
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

    Contours           contours;
    Contours      filt_contours;
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

    // Let's filter out contours that are too small to be an object
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // ROS_INFO("IDX %lu Col %s Contour area %g", i, col.toString().c_str(),
        //                                            contourArea(contours[i], false));
        if( int(contourArea(contours[i], false)) > area_threshold )
        {
            filt_contours.push_back(contours[i]);
        }
    }

    // If there no contours any more, the object is not there
    if (filt_contours.size() == 0)
    {
        is_there = false;
        return false;
    }

    // At this stage, there should be only a contour in the vector of contours.
    // If this is not the case, let's pick the contour with the biggest area
    int largest_area=0;
    int largest_contour_idx=-1;

    for( size_t i = 0; i< filt_contours.size(); i++ )
    {
        double a = contourArea(filt_contours[i], false);  //  Find the area of contour

        if( a > largest_area )
        {
            largest_area = a;
            largest_contour_idx = i;                // Store the index of largest contour
        }
    }

    rect = minAreaRect(cv::Mat(filt_contours[largest_contour_idx]));

    cv::Scalar color = cv::Scalar::all(255);

    cv::Point2f rect_points[4];
    rect.points(rect_points);

    for( int j = 0; j < 4; j++ )
    {
        cv::line   (_out, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        cv::putText(_out, intToString(j), rect_points[j],
                     cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar::all(255), 2, CV_AA);
    }

    is_there = true;
    return true;
}

SegmentedObjHSV::~SegmentedObjHSV()
{

}

/************************************************************************************/
/*                             CARTESIAN ESTIMATOR HSV                              */
/************************************************************************************/
CartesianEstimatorHSV::CartesianEstimatorHSV(string  _name, vector<string> _objs_name,
                                             cv::Mat _objs_size, vector<hsvColorRange> _objs_col) :
                                             CartesianEstimator(_name)
{
    ROS_ASSERT_MSG(_objs_size.cols == 2, "Objects' sizes should have two columns. "
                   "%i found instead", _objs_size.cols);

    objsFromMat(_objs_name, _objs_size, _objs_col);
}

bool CartesianEstimatorHSV::addObject(string _name, double _h, double _w, hsvColorRange _hsv)
{
    vector<double> size;

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

    objs.push_back(new SegmentedObjHSV(_name, size, getAreaThreshold(), _hsv));

    return true;
}

bool CartesianEstimatorHSV::objsFromMat(vector<string> _names, cv::Mat _o,
                                        vector<hsvColorRange> _hsvs)
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
        res = res && addObject(_names[i], _o.at<float>(i, 0), _o.at<float>(i, 1), _hsvs[i]);
    }

    return res;
}

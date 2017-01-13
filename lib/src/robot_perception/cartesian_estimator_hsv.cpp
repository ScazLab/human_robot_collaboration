#include "robot_perception/cartesian_estimator_hsv.h"

using namespace std;

SegmentedObjHSV::SegmentedObjHSV(std::vector<double> _size, hsvColorRange _col) :
                                 SegmentedObj(_size), col(_col)
{

}

bool SegmentedObjHSV::detectObject(const cv::Mat& _in, cv::Mat& _out)
{
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

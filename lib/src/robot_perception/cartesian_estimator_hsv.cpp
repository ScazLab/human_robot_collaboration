#include "robot_perception/cartesian_estimator_hsv.h"

using namespace std;

SegmentedObjHSV::SegmentedObjHSV(std::vector<double> _size, hsvColorRange _col) :
                                 SegmentedObj(_size), col(_col)
{

}

CartesianEstimatorHSV::CartesianEstimatorHSV(std::string _name, cv::Mat _objs_size,
                                             std::vector<hsvColorRange> _objs_col) :
                                             CartesianEstimator(_name)
{

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

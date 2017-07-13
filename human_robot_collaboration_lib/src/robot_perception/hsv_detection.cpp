#include "robot_perception/hsv_detection.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

cv::Mat hsvThreshold(const cv::Mat& _src, hsvColorRange _hsv)
{
    cv::Mat res = _src.clone();

    // If H.lower is higher than H.upper it means that we would like to
    // detect something in the range [0-upper] & [lower-180] (i.e. the red)
    // So the thresholded image will be made with two opencv calls to inRange
    // and then the two will be merged into one
    if (_hsv.H.min > _hsv.H.max)
    {
        cv::Mat resA = _src.clone();
        cv::Mat resB = _src.clone();

        cv::inRange(_src, cv::Scalar(         0, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(_hsv.H.max, _hsv.S.max, _hsv.V.max), resA);
        cv::inRange(_src, cv::Scalar(_hsv.H.min, _hsv.S.min, _hsv.V.min),
                          cv::Scalar(       180, _hsv.S.max, _hsv.V.max), resB);

        cv::bitwise_or(resA,resB,res);
    }
    else
    {
        cv::inRange(_src, cv::Scalar(_hsv.get_hsv_min()),
                          cv::Scalar(_hsv.get_hsv_max()), res);
    }

    return res;
}

/**************************************************************************/
/**                        COLOR_RANGE                                   **/
/**************************************************************************/

colorRange::colorRange(XmlRpc::XmlRpcValue _cR)
{
    min = static_cast<int>(_cR[0]);
    max = static_cast<int>(_cR[1]);
}

colorRange & colorRange::operator=(const colorRange &_cr)
{
    min=_cr.min;
    max=_cr.max;
    return *this;
}

/**************************************************************************/
/**                         HSV_COLOR                                    **/
/**************************************************************************/

hsvColorRange::hsvColorRange(XmlRpc::XmlRpcValue _H,
                             XmlRpc::XmlRpcValue _S, XmlRpc::XmlRpcValue _V)
{
    H = colorRange(_H);
    S = colorRange(_S);
    V = colorRange(_V);
}

hsvColorRange::hsvColorRange(XmlRpc::XmlRpcValue _params)
{
    ROS_ASSERT(_params.getType()==XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator i=_params.begin(); i!=_params.end(); ++i)
    {
        ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
        for(int j=0; j<i->second.size(); ++j)
        {
            ROS_ASSERT(i->second[j].getType()==XmlRpc::XmlRpcValue::TypeInt);
        }
        // printf("%s %i %i\n", i->first.c_str(), static_cast<int>(i->second[0]),
        //                                        static_cast<int>(i->second[1]));
        if (i->first == "H") H = colorRange(i->second);
        if (i->first == "S") S = colorRange(i->second);
        if (i->first == "V") V = colorRange(i->second);
    }
}

string hsvColorRange::toString()
{
    stringstream res;
    res <<"H=["<<H.min<<"\t"<<H.max<<"]\t"
        <<"S=["<<S.min<<"\t"<<S.max<<"]\t"
        <<"V=["<<V.min<<"\t"<<V.max<<"]";
    return res.str();
}

hsvColorRange & hsvColorRange::operator=(const hsvColorRange &_hsvc)
{
    H=_hsvc.H;
    S=_hsvc.S;
    V=_hsvc.V;
    return *this;
}

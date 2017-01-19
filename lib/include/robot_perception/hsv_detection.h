#ifndef __HSV_DETECTION_H__
#define __HSV_DETECTION_H__

#include <ros/ros.h>
#include <opencv2/core/core.hpp>

typedef std::vector<cv::Point>   Contour;
typedef std::vector<Contour>    Contours;

struct colorRange
{
    int min;
    int max;

    colorRange(): min(0), max(0) {};
    colorRange(int _min, int _max) : min(_min), max(_max) {};
    colorRange(const colorRange &_cr): min(_cr.min), max(_cr.max) {};

    /**
     * Reads the colorRange values as an XmlRpcValue::TypeArray from the parameter server.
     *
     * @param  _cR the TypeArray read from the parameter server.
     */
    colorRange(XmlRpc::XmlRpcValue _cR);

    /**
    * Copy Operator
    **/
    colorRange &operator=(const colorRange &);
};

struct hsvColorRange
{
    colorRange H;
    colorRange S;
    colorRange V;

    /* CONSTRUCTORS */
    hsvColorRange(): H(0,180), S(0,256), V(0,256) {};
    hsvColorRange(const colorRange &_H, const colorRange &_S,
                  const colorRange &_V) : H(_H), S(_S), V(_V) {};

    /**
     * Reads the HSV values as an XmlRpcValue::TypeArray from the parameter server.
     *
     * @param  _H the H TypeArray read from the parameter server.
     * @param  _S the S TypeArray read from the parameter server.
     * @param  _V the V TypeArray read from the parameter server.
     */
    hsvColorRange(XmlRpc::XmlRpcValue _H, XmlRpc::XmlRpcValue _S, XmlRpc::XmlRpcValue _V);

    /**
     * Reads the HSV values as an XmlRpcValue::TypeStruct from the parameter server.
     * It is encoded as an entire namespace of parameters using a YAML dictionary.
     * This is a valid parameter to set in your launch file:
     *
     * <rosparam param = "ttt_controller/hsv_red">
     *   H: [160,  20]
     *   S: [ 40, 196]
     *   V: [ 50, 196]
     * </rosparam>
     *
     * @param  _param the XmlRpcValue read from the parameter server.
     */
    hsvColorRange(XmlRpc::XmlRpcValue _params);

    cv::Scalar get_hsv_min() { return cv::Scalar(H.min, S.min, V.min); };
    cv::Scalar get_hsv_max() { return cv::Scalar(H.max, S.max, V.max); };

    /**
     * Print function.
     *
     * @return A text description of the cell
     */
    std::string toString();

    /**
    * Copy Operator
    **/
    hsvColorRange &operator=(const hsvColorRange &);
};

/**
 * Thresholds the HSV image and create a binary image. It can automatically
 * take care of the fact that the red is at the extremes of the HSV spectrum,
 * so two thresholds may be needed in order to robustly detect the red color.
 *
 * @param  _src [description]
 * @param  _hsv [description]
 * @return      [description]
 */
cv::Mat hsvThreshold(const cv::Mat& _src, hsvColorRange _hsv);


#endif // __HSV_DETECTION_H__

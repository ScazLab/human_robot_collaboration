#ifndef __CARTESIAN_ESTIMATOR_HSV__
#define __CARTESIAN_ESTIMATOR_HSV__

#include "robot_perception/hsv_detection.h"
#include "robot_perception/cartesian_estimator.h"

class SegmentedObjHSV: public SegmentedObj
{
public:

    // Color of the object to segment
    hsvColorRange col;

    /* CONSTRUCTOR */
    SegmentedObjHSV(std::vector<double> _size, hsvColorRange _col);
    SegmentedObjHSV(std::string _name, std::vector<double> _size, int _area_thres, hsvColorRange _col);

    /* DESTRUCTOR */
    ~SegmentedObjHSV();

    /**
     * Detects the object in the image
     *
     * @param _in        Input image to detect objects from
     * @param _out       Output image to show the result of the segmentation
     *
     * @return true/false if success/failure
     */
    bool detectObject(const cv::Mat& _in, cv::Mat& _out);

    /**
     * Detects the object in the image
     *
     * @param _in        Input image to detect objects from
     * @param _out       Output image to show the result of the segmentation
     * @param _out_thres Optional output image to show the thresholded image
     *
     * @return true/false if success/failure
     */
    bool detectObject(const cv::Mat& _in, cv::Mat& _out, cv::Mat& _out_thres);

    /**
     * Converts the segmented object to a string.
     * @return the segmented object as a string
     */
    std::string toString();
};

/**
 * Class that is able to detect objects from a range of HSV colors that
 * define their color.
 */
class CartesianEstimatorHSV : public CartesianEstimator
{
protected:

    /**
     * Adds an object to the database of objects.
     *
     * @param h Its physical height in meters
     * @param w Its physical width in meters
     *
     * @return true/false if success/failure
     */
    bool addObject(std::string _name, double _h, double _w, hsvColorRange _hsv);

    /**
     * Adds the full database of objects from a cv::Mat
     *
     * @param _o the database of objects
     * @return true/false if success/failure
     */
    bool addObjects(std::vector<std::string> _names, cv::Mat _o,
                    std::vector<hsvColorRange> _hsvs);

    /**
     * Creates the objects database from an XmlRpcValue::TypeStruct in the parameter server.
     * It is encoded as an entire namespace of parameters using a YAML dictionary.
     * The format is an array of arrays composed of a string (the name of the obejct),
     * and the width and heigth of the object, H, S, V as other nested arrays.
     * This is a valid parameter to set in your launch file:
     *
     * <rosparam param = "cartesian_estimator/objects">
     *    [["screwdriver", [0.0525, 0.015], [160,  10], [70, 166], [10,  66]],
     *     ["blue_box"   ,  [0.111, 0.052], [ 60, 130], [90, 256], [10, 256]]]
     * </rosparam>
     *
     * @param  _param the XmlRpcValue read from the parameter server.
     * @return        true/false if the insertion was successful or not
     */
    bool addObjects(XmlRpc::XmlRpcValue _params);

    /**
     * Detects the object in the image
     */
    bool detectObjects(const cv::Mat& _in, cv::Mat& _out) { return false; };


public:
    /* CONSTRUCTORS */
    CartesianEstimatorHSV(std::string  _name);
};

#endif

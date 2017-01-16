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
    bool objsFromMat(std::vector<std::string> _names, cv::Mat _o,
                     std::vector<hsvColorRange> _hsvs);

    /**
     * Detects the object in the image
     */
    bool detectObjects(const cv::Mat& _in, cv::Mat& _out) { return false; };


public:
    /* CONSTRUCTORS */
    CartesianEstimatorHSV(std::string  _name, std::vector<std::string> _objs_name,
                          cv::Mat _objs_size, std::vector<hsvColorRange> _objs_col);
};

#endif

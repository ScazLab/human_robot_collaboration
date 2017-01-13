#ifndef __CARTESIAN_ESTIMATOR_HSV__
#define __CARTESIAN_ESTIMATOR_HSV__

#include "robot_perception/hsv_detection.h"
#include "robot_perception/cartesian_estimator.h"

struct SegmentedObjHSV: public SegmentedObj
{
    // Color of the object to segment
    hsvColorRange col;

    SegmentedObjHSV(std::vector<double> _size, hsvColorRange _col);
};

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
    bool addObject(double _h, double _w, hsvColorRange _hsv);

    /**
     * Adds the full database of objects from a cv::Mat
     *
     * @param _o the database of objects
     * @return true/false if success/failure
     */
    bool objsFromMat(cv::Mat _o, std::vector<hsvColorRange> _hsvs);

public:
    /* CONSTRUCTORS */
    CartesianEstimatorHSV(std::string _name, cv::Mat _objs_size, std::vector<hsvColorRange> _objs_col);
};

#endif

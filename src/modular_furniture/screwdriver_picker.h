#include <stdio.h>

#include <ros/ros.h>
#include "robot_perception/cartesian_estimator_hsv.h"

class ScrewDriverPicker : public CartesianEstimatorHSV
{
protected:
    /**
     * Detects the screwdriver in the image
     */
    bool detectObject(const cv::Mat& _in, cv::Mat& _out);

public:
    ScrewDriverPicker(std::string _name, cv::Mat _objs_size, std::vector<hsvColorRange> _objs_col);
};

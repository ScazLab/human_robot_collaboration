#include <stdio.h>

#include <ros/ros.h>
#include "robot_perception/hsv_detection.h"
#include "robot_perception/cartesian_estimator.h"

class ScrewDriverPicker : public CartesianEstimator
{
protected:
    /**
     * Detects the screwdriver in the image
     */
    bool detectObject(const cv::Mat& _in, cv::Mat& _out);

public:
    ScrewDriverPicker(std::string _name, cv::Mat _objs_size);
};

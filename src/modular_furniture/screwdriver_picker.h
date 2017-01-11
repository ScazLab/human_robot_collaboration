#include <stdio.h>

#include <ros/ros.h>
#include "robot_perception/cartesian_estimator.h"

class ScrewDriverPicker : public CartesianEstimator
{
protected:
    /**
     * Detects the screwdriver in the image
     */
    void detectObject(const cv::Mat& _in, cv::Mat& _out);

public:
    ScrewDriverPicker(std::string name, std::vector<double> _obj_size);
};

#ifndef __ROS_THREAD_IMAGE_H__
#define __ROS_THREAD_IMAGE_H__

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <pthread.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <baxter_core_msgs/DigitalIOState.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/CollisionAvoidanceState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>

#include "robot_utils/utils.h"
#include "robot_interface/ros_thread.h"

/**
 * @brief A ROS Thread with an image callbck
 * @details This class inherits from ROSThread, but it adds also an image callback
 *          to be overwritten by its children. Useful to to visual processing.
 */
class ROSThreadImage : public ROSThread
{
private:
    std::string    _name;

    image_transport::ImageTransport _img_trp;
    image_transport::Subscriber     _img_sub;

    pthread_mutex_t _mutex_img;

    ros::AsyncSpinner spinner;  // AsyncSpinner to handle callbacks

protected:
    ros::NodeHandle _n;

    cv::Mat  _curr_img;
    cv::Size _img_size;
    bool    _img_empty;

    /*
     * Function that will be spun out as a thread
     */
    virtual void InternalThreadEntry() = 0;

public:
    ROSThreadImage(std::string name);
    ~ROSThreadImage();

    /*
     * image callback function that displays the image stream from the hand camera
     *
     * @param      The image
     * @return     N/A
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /*
     * Self-explaining "setters"
     */
    void setName(std::string name) { _name = name; };

    /*
     * Self-explaining "getters"
     */
    std::string  getName() { return  _name; };
};

#endif

#ifndef __ROS_THREAD_IMAGE_H__
#define __ROS_THREAD_IMAGE_H__

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "robot_utils/utils.h"

#include <mutex>
#include <thread>

/**
 * @brief A ROS Thread with an image callback
 * @details This class inherits from ROSThread, but it adds also an image callback
 *          to be overwritten by its children. Useful to to visual processing.
 */
class ROSThreadImage
{
protected:
    ros::NodeHandle nh;

private:
    std::string       name;
    std::thread img_thread;

    bool           is_closing;  // Flag to close the thread entry function
    std::mutex mtx_is_closing;  // Mutex to protect the thread close flag

    ros::AsyncSpinner spinner;  // AsyncSpinner to handle callbacks

protected:
    image_transport::ImageTransport img_trp;
    image_transport::Subscriber     img_sub;

    std::mutex mutex_img;

    cv::Mat     curr_img;   // Current image
    cv::Size    img_size;   // Size of current image
    bool       img_empty;   // Returns true if current image is empty, false otherwise
    std::string encoding;   // Encoding for the image read by the subscriber

    ros::Rate r;

    /*
     * Function that will be spun out as a thread
     */
    virtual void internalThread() = 0;

public:
    /**
     * Constructor
     *
     * @param _name     name of the object
     * @param _encoding encoding for the image
     */
    ROSThreadImage(std::string _name, std::string _encoding = "bgr8");

    /**
     * Destructor
     */
    ~ROSThreadImage();

    /*
     * image callback function that displays the image stream from the image topic
     *
     * @param      The image
     * @return     N/A
     */
    void imageCb(const sensor_msgs::ImageConstPtr& _msg);

    /*
     * Self-explaining "setters"
     */
    void setName(std::string _name) { name = _name; };

    /*
     * Self-explaining "getters"
     */
    std::string getName()     { return     name; };
    std::string getEncoding() { return encoding; };

    /*
     * Starts thread
     */
    bool startThread();

    /**
     * Safely manipulate the boolean needed to kill the thread entry
     */
    void setIsClosing(bool _arg);
    bool isClosing();
};

#endif

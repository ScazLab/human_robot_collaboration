#include "robot_utils/ros_thread_image.h"

/**************************************************************************/
/*                          ROSThreadImage                                */
/**************************************************************************/

ROSThreadImage::ROSThreadImage(std::string _name) :  _n(_name), name(_name),
                                     spinner(4), _img_trp(_n), _img_empty(true), r(30) // 30Hz
{

    _img_sub = _img_trp.subscribe("/"+getName()+"/image", // "/cameras/right_hand_camera/image",
                           SUBSCRIBER_BUFFER, &ROSThreadImage::imageCb, this);

    spinner.start();
    startThread();
}

bool ROSThreadImage::startThread()
{
    img_thread = std::thread(&ROSThreadImage::InternalThreadEntry, this);
    return img_thread.joinable();
}

void ROSThreadImage::setIsClosing(bool arg)
{
    std::lock_guard<std::mutex> lock(mtx_is_closing);
    is_closing = arg;
}

bool ROSThreadImage::isClosing()
{
    std::lock_guard<std::mutex> lock(mtx_is_closing);
    return is_closing;
}

ROSThreadImage::~ROSThreadImage()
{
    setIsClosing(true);
    if (img_thread.joinable()) { img_thread.join(); }
}

void ROSThreadImage::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("imageCb");
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_img);
    _curr_img  = cv_ptr->image.clone();
    _img_size  =      _curr_img.size();
    _img_empty =     _curr_img.empty();
}


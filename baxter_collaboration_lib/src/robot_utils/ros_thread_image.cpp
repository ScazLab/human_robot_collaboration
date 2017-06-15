#include "robot_utils/ros_thread_image.h"

/**************************************************************************/
/*                          ROSThreadImage                                */
/**************************************************************************/

ROSThreadImage::ROSThreadImage(std::string _name, std::string _encoding) :
                               nh(_name), name(_name), is_closing(false),
                               spinner(4), img_trp(nh), img_empty(true),
                               encoding(_encoding), r(50) // 50Hz
{
    img_sub = img_trp.subscribe("/"+getName()+"/image", // "/cameras/right_hand_camera/image",
                                  SUBSCRIBER_BUFFER, &ROSThreadImage::imageCb, this);

    spinner.start();
}

bool ROSThreadImage::startThread()
{
    img_thread = std::thread(&ROSThreadImage::InternalThreadEntry, this);
    return img_thread.joinable();
}

void ROSThreadImage::setIsClosing(bool _arg)
{
    std::lock_guard<std::mutex> lock(mtx_is_closing);
    is_closing = _arg;
}

bool ROSThreadImage::isClosing()
{
    std::lock_guard<std::mutex> lock(mtx_is_closing);
    return is_closing;
}

void ROSThreadImage::imageCb(const sensor_msgs::ImageConstPtr& _msg)
{
    // ROS_INFO("imageCb");
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(_msg, encoding);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_img);
    curr_img  = cv_ptr->image.clone();
    img_size  =       curr_img.size();
    img_empty =      curr_img.empty();
}

ROSThreadImage::~ROSThreadImage()
{
    setIsClosing(true);

    if (img_thread.joinable()) { img_thread.join(); }
}


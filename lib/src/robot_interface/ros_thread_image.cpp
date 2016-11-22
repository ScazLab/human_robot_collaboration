#include "robot_interface/ros_thread_image.h"

using namespace std;

/**************************************************************************/
/*                          ROSThreadImage                                */
/**************************************************************************/

ROSThreadImage::ROSThreadImage(string name) :  ROSThread(), _name(name), _n(name),
                                               _img_trp(_n), spinner(4), _img_empty(true)
{
    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init(&_mutex_img, &_mutex_attr);

    _img_sub = _img_trp.subscribe("/"+getName()+"/image",
                           SUBSCRIBER_BUFFER, &ROSThreadImage::imageCb, this);

    spinner.start();
    startInternalThread();
}

ROSThreadImage::~ROSThreadImage()
{
    closeInternalThread();
    pthread_mutex_destroy(&_mutex_img);
}

void ROSThreadImage::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("imageCb");
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("[Arm Controller] cv_bridge exception: %s", e.what());
        return;
    }

    pthread_mutex_lock(&_mutex_img);
    _curr_img  = cv_ptr->image.clone();
    _img_size  =      _curr_img.size();
    _img_empty =     _curr_img.empty();
    pthread_mutex_unlock(&_mutex_img);
}


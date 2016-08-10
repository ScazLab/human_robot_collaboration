#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__

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
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>

#include "utils.h"

/**
 * @brief A Thread class
 * @details This class initializes overhead functions necessary to start a thread
 *          from within a class
 */
class Thread
{
private:
    pthread_t _thread;
    static void * InternalThreadEntryFunc(void * This);

protected:
    /*
     * Function that will be spun out as a thread
     */
    virtual void InternalThreadEntry() = 0;

    /*
     * Prevents any following code from being executed before thread is exited
     * @return true/false if success/failure (not in the POSIX way)
     */      
    void waitForInternalThreadToExit();

public:
    /*
     * Constructor
    */
    Thread();

    /*
     * Starts thread that executes the internal thread entry function
     * 
     * @param      N/A
     * @return     true if thread was successfully launched; false otherwise
     */        
    bool startInternalThread();

    /**
     * Kills the internal thread
     * @return true/false if success/failure (not in the POSIX way)
     */
    bool killInternalThread();

    /*
     * Destructor
    */
    virtual ~Thread();
    
};

/**
 * @brief A ROS Thread class
 * @details This class initializes overhead ROS features: subscriber/publishers,
 *          services, callback functions etc.
 */
class ROSThread : public Thread
{
private:
    std::string    _limb;
    State         _state;
    ros::Time _init_time;

    ros::AsyncSpinner spinner;

    ros::ServiceClient _ik_client;

    ros::Publisher  _joint_cmd_pub;
    ros::Publisher    _coll_av_pub;

    // IR Sensor
    ros::Subscriber _ir_sub;
    bool              ir_ok;
    float       _curr_range;
    float   _curr_min_range;
    float   _curr_max_range;

    // End-Effector
    ros::Subscriber            _endpt_sub;
    std::vector<double>       _filt_force;
    double                    force_thres;

    geometry_msgs::Point        _curr_pos;
    geometry_msgs::Quaternion   _curr_ori;
    geometry_msgs::Wrench    _curr_wrench;

protected:
    /*
     * Function that will be spun out as a thread
     */
    virtual void InternalThreadEntry() = 0;

    ros::NodeHandle _n;

    // Cuff OK Button (the circular one)
    ros::Subscriber _cuff_sub;

    /*
     * Checks for if the system is ok. To be called inside every thread execution,
     * in order to make it exit gracefully if there is any problem.
     * It also checks for the ROS state.
     * @return true if everything is okay, false otherwise
     */
    bool ok();

    /*
     * checks if end effector has made contact with a token by checking if 
     * the range of the infrared sensor has fallen below the threshold value
     * 
     * @param      current range values of the IR sensor, and a string 
     *            (strict/loose) indicating whether to use a high or low
     *            threshold value
     *             
     * return     true if end effector has made contact; false otherwise
     */
    bool hasCollided(std::string mode = "loose");

    /*
     * checks if the arm has completed its intended move by comparing
     * the requested pose and the current pose
     *
     * @param  p     requested pose
     * @param  mode  string (strict/loose) indicating the desired level of accuracy
     * @return       true/false if success/failure
     */
    bool hasPoseCompleted(double px, double py, double pz,
                          double ox, double oy, double oz, double ow, std::string mode = "loose");

    /*
     * Uses built in IK solver to find joint angles solution for desired pose
     * 
     * @param     requested PoseStamped
     * @param     array of joint angles solution
     * @return    true/false if success/failure
     */
    bool callIKService(double px, double py, double pz,
                       double ox, double oy, double oz, double ow,
                       std::vector<double>& joint_angles);

    /*
     * Moves arm to the requested pose. This differs from ROSThread::goToPose because it 
     * does not check if the final pose has been reached, but rather it goes in open-loop
     * unitil a fisical contact with the table is reached
     * 
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @return true/false if success/failure
     */
    bool goToPoseNoCheck(double px, double py, double pz,
                         double ox, double oy, double oz, double ow);

    bool goToPoseNoCheck(std::vector<double> joint_angles);

    /*
     * Moves arm to the requested pose , and checks if the pose has been achieved
     * 
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @param  mode (either loose or strict, it checks for the final desired position)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow,
                  std::string mode="loose", bool disable_coll_av = false);

    /*
     * Sets the joint names of a JointCommand
     * 
     * @param    joint_cmd the joint command
     */
    void setJointNames(baxter_core_msgs::JointCommand& joint_cmd);

    /*
     * Detects if the force overcame a set threshold in either one of its three axis
     * 
     * @return true/false if the force overcame the threshold
     */
    bool detectForceInteraction();

    /*
     * Waits for a force interaction to occur.
     * 
     * @return true when the force interaction occurred
     * @return false if no force interaction occurred after 20s
     */
    bool waitForForceInteraction(double _wait_time = 20.0, bool disable_coll_av = false);

    /*
     * Callback function that sets the current pose to the pose received from 
     * the endpoint state topic
     * 
     * @param      N/A
     * @return     N/A
     */
    void endpointCb(const baxter_core_msgs::EndpointState& msg);

    /*
     * Callback function for the CUFF OK button
     * 
     * @param      N/A
     * @return     N/A
     */
    void cuffOKCb(const baxter_core_msgs::DigitalIOState& msg);

    /*
     * Infrared sensor callback function that sets the current range to the range received
     * from the left hand range state topic
     * 
     * @param      The message
     * @return     N/A
     */
    void IRCb(const sensor_msgs::RangeConstPtr& msg);

    /*
     * Filters the forces with a very simple low pass filter
     */
    void filterForces();

    /*
     * hover arm above tokens
     * 
     * @param      double indicating requested height of arm (z-axis)
     * return     N/A
     */
    void hoverAboveTokens(double height);

    /**
     * @brief Publishes the desired joint configuration
     * @details Publishes the desired joint configuration in the proper topic
     * 
     * @param _cmd The desired joint configuration
     */
    void publish_joint_cmd(baxter_core_msgs::JointCommand _cmd);

    /**
     * @brief Suppresses the collision avoidance for this arm
     * @details Suppresses the collision avoidance. It needs to be called with 
     *          a rate of at least 5Hz
     * 
     * @param _cmd An empty message to be sent
     */
    void suppressCollisionAv();

public:
    ROSThread(std::string limb);
    
    virtual ~ROSThread();

    /*
     * Self-explaining "setters"
     */   
    void setState(int state);

    /*
     * Self-explaining "getters"
     */
    State       getState() { return _state; };
    std::string getLimb()  { return  _limb; };

    geometry_msgs::Point        getPos()    { return    _curr_pos; };
    geometry_msgs::Quaternion   getOri()    { return    _curr_ori; };
    geometry_msgs::Wrench       getWrench() { return _curr_wrench; };

    /*
     * Check availability of the infrared data
    */
    bool    is_ir_ok() { return ir_ok; };
};

/**
 * @brief A ROS Thread with an image callbck
 * @details This class inherits from ROSThread, but it adds also an image callback
 *          to be overwritten by its children. Useful to to visual processing.
 */
class ROSThreadImage : public ROSThread
{
private:
    image_transport::ImageTransport _img_trp;
    image_transport::Subscriber _img_sub;

protected:
    cv::Mat _curr_img;
    cv::Size _curr_img_size;
    bool _curr_img_empty;

    pthread_mutex_t _mutex_img;

public:
    ROSThreadImage(std::string limb);
    ~ROSThreadImage();

    /*
     * image callback function that displays the image stream from the hand camera 
     * 
     * @param      The image
     * @return     N/A
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif

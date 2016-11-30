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

#include "robot_utils/utils.h"

/**
 * @brief A Thread class
 * @details This class initializes overhead functions necessary to start a thread
 *          from within a class
 */
class ROSThread
{
private:
    pthread_t _thread;
    bool   is_started;
    static void * InternalThreadEntryFunc(void * obj);

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
    ROSThread();

    /*
     * Starts thread that executes the internal thread entry function
     *
     * @return  true/false if success failure (NOT in the POSIX way)
     */
    bool startInternalThread();

    /**
     * Closes the internal thread gracefully.
     *
     * @return  true/false if success failure (NOT in the POSIX way)
     */
    void closeInternalThread();

    /**
     * Kills the internal thread.
     *
     * @return true/false if success/failure (NOT in the POSIX way)
     */
    bool killInternalThread();

    /*
     * Destructor
    */
    ~ROSThread();

};

#endif

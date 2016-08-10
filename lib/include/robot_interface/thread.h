#ifndef __THREAD_H__
#define __THREAD_H__

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

#endif

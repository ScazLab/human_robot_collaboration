#ifndef __ROS_THREAD_OBJ_H__
#define __ROS_THREAD_OBJ_H__

#include <pthread.h>

#include "robot_utils/utils.h"

/**
 * @brief A Thread class
 * @details This class wraps overhead functions necessary to start a thread
 * from within a class. It is not a virtual class, and it can be used as an
 * an independent object.
 */
class ROSThreadObj
{
private:
    pthread_t _thread;
    bool   is_started;

    ros::Rate r;

public:
    /*
     * Constructor
    */
    ROSThreadObj(double _rate = 100);

    /**
     * Sleeps the amount of time defined by r (by default, r is set to 100Hz)
     */
    void sleep();

    /*
     * Starts thread that executes the internal thread entry function
     *
     * @return  true/false if success failure (NOT in the POSIX way)
     */
    bool start(void *(*ThreadFunction)(void*) );

    /*
     * Prevents any following code from being executed before thread is exited
     * @return true/false if success/failure (not in the POSIX way)
     */
    void join();

    /**
     * Closes the internal thread gracefully.
     *
     * @return  true/false if success failure (NOT in the POSIX way)
     */
    void close();

    /**
     * Kills the internal thread.
     *
     * @return true/false if success/failure (NOT in the POSIX way)
     */
    bool kill();

    /*
     * Destructor
    */
    ~ROSThreadObj();

};

#endif

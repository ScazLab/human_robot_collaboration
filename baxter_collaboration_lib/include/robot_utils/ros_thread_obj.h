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

public:
    /*
     * Constructor
    */
    ROSThreadObj();

    /*
     * Starts thread that executes the internal thread entry function
     *
     * @param ThreadFunction  the function that is going to be run in the thread
     * @param arg             arg is passed as the sole argument of ThreadFunction.
     *                        This differs from ROSThread because in there we were
     *                        ok to use 'this' as arg. In this case, since this class
     *                        should be used as a member object of another class, we
     *                        need to explicitly look for an arg that makes sense.
     *                        Typical use would be start(ThreadEntryFunc, this), so
     *                        that this class can access the main class members.
     * @return  true/false if success failure (NOT in the POSIX way)
     */
    bool start(void *(*ThreadFunction)(void*), void *arg);

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
    bool close();

    /**
     * Kills the internal thread.
     *
     * @return true/false if success/failure (NOT in the POSIX way)
     */
    bool kill();

    /**
     * @return true/false if the thread is running or not.
     */
    bool is_running() { return is_started; };

    /*
     * Destructor
    */
    ~ROSThreadObj();

};

#endif

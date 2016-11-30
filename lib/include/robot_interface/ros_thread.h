#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__

#include <pthread.h>

/**
 * @brief A Thread class
 * @details This class wraps overhead functions necessary to start a thread
 * from within a class. It is a virtual class, and  its InternalThreadEntry
 * method needs to be implemented in its children. Please see ROSThreadImpl
 * for a class that can be used as an independent object.
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

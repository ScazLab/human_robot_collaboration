#include "robot_interface/ros_thread.h"

/**************************************************************************/
/*                             ROSThread                                  */
/**************************************************************************/
ROSThread::ROSThread(): is_started(false) { }

void * ROSThread::InternalThreadEntryFunc(void * obj)
{
    ((ROSThread *)obj)->InternalThreadEntry();
    return NULL;
}

bool ROSThread::startInternalThread()
{
    is_started = pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0;
    return is_started;
}

void ROSThread::joinInternalThread()
{
    (void) pthread_join(_thread, NULL);
}

bool ROSThread::closeInternalThread()
{
    if (is_started)
    {
        is_started = false;
        pthread_exit(NULL);
        return true;
    }

    return false;
}

bool ROSThread::killInternalThread()
{
    if (is_started)
    {
        is_started = false;
        return pthread_cancel(_thread) == 0;
    }

    return false;
}

ROSThread::~ROSThread() { }

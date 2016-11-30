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

void ROSThread::closeInternalThread()
{
    pthread_exit(NULL);
    is_started = false;
    return;
}

bool ROSThread::killInternalThread()
{
    if (is_started) return pthread_cancel(_thread) == 0;
    else            return false;
}

ROSThread::~ROSThread() { }

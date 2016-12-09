#include "robot_utils/ros_thread_obj.h"

/**************************************************************************/
/*                           ROSThreadObj                                 */
/**************************************************************************/
ROSThreadObj::ROSThreadObj(): is_started(false) { }

bool ROSThreadObj::start(void *(*ThreadFunction)(void*), void *arg)
{
    is_started = pthread_create(&_thread, NULL, ThreadFunction, arg) == 0;
    return is_started;
}

void ROSThreadObj::join()
{
    (void) pthread_join(_thread, NULL);
}

bool ROSThreadObj::close()
{
    if (is_started)
    {
        is_started = false;
        pthread_exit(NULL);
        return true;
    }

    return false;
}

bool ROSThreadObj::kill()
{
    if (is_started)
    {
        is_started = false;
        return pthread_cancel(_thread) == 0;
    }

    return false;
}

ROSThreadObj::~ROSThreadObj() { }

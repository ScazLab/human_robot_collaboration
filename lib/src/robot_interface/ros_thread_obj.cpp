#include "robot_interface/ros_thread_obj.h"

/**************************************************************************/
/*                           ROSThreadObj                                 */
/**************************************************************************/
ROSThreadObj::ROSThreadObj(double _rate): is_started(false), r(_rate) { }

bool ROSThreadObj::start(void *(*ThreadFunction)(void*) )
{
    is_started = pthread_create(&_thread, NULL, ThreadFunction, this) == 0;
    return is_started;
}

void ROSThreadObj::sleep()
{
    r.sleep();
    return;
}

void ROSThreadObj::join()
{
    (void) pthread_join(_thread, NULL);
}

void ROSThreadObj::close()
{
    pthread_exit(NULL);
    is_started = false;
    return;
}

bool ROSThreadObj::kill()
{
    if (is_started) return pthread_cancel(_thread) == 0;
    else            return false;
}

ROSThreadObj::~ROSThreadObj() { }

#include "robot_interface/thread.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace cv;

/**************************************************************************/
/*                             Thread                                     */
/**************************************************************************/
Thread::Thread() { }

void * Thread::InternalThreadEntryFunc(void * This)
{
    ((Thread *)This)->InternalThreadEntry(); 
    return NULL;
}

bool Thread::startInternalThread()
{
    return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
}

void Thread::waitForInternalThreadToExit()
{
    (void) pthread_join(_thread, NULL);
}

bool Thread::killInternalThread()
{
    return (pthread_cancel(_thread) == 0);
}

Thread::~Thread() { }

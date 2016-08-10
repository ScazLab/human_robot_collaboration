#include "robot_interface/ros_thread.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace cv;

/**************************************************************************/
/*                             ROSThread                                  */
/**************************************************************************/
ROSThread::ROSThread() { }

void * ROSThread::InternalThreadEntryFunc(void * This)
{
    ((ROSThread *)This)->InternalThreadEntry(); 
    return NULL;
}

bool ROSThread::startInternalThread()
{
    return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
}

void ROSThread::waitForInternalThreadToExit()
{
    (void) pthread_join(_thread, NULL);
}

bool ROSThread::killInternalThread()
{
    return (pthread_cancel(_thread) == 0);
}

ROSThread::~ROSThread() { }

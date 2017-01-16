#include "robot_perception/cartesian_estimator_client.h"

using namespace std;

CartesianEstimatorClient::CartesianEstimatorClient(string name, string limb) :
                        _nh(name), _limb(limb), cartest_ok(false),
                        object_found(false), object_id(-1)
{
    _cartest_sub = _nh.subscribe("/hsv_detector/objects", SUBSCRIBER_BUFFER,
                                  &CartesianEstimatorClient::ObjectCb, this);
}

void CartesianEstimatorClient::clearMarkerPose()
{
    cartest_ok     = false;
    object_found = false;
}

void CartesianEstimatorClient::ObjectCb(const baxter_collaboration::ObjectsArray& msg)
{
    for (size_t i = 0; i < msg.objects.size(); ++i)
    {
        // ROS_DEBUG("Processing object with id %i",msg.objects[i].id);

        if (int(msg.objects[i].id) == getMarkerID())
        {
            _curr_object_pos = msg.objects[i].pose.position;
            _curr_object_ori = msg.objects[i].pose.orientation;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_object_pos.x,
                                                _curr_object_pos.y,
                                                _curr_object_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", _curr_object_ori.x,
            //                                       _curr_object_ori.y,
            //                                       _curr_object_ori.z,
            //                                       _curr_object_ori.w);

            if (!object_found)
            {
                object_found = true;
            }
        }
    }

    if (!cartest_ok)
    {
        cartest_ok = true;
    }
}

bool CartesianEstimatorClient::waitForARucoData()
{
    clearMarkerPose();
    ROS_INFO("[%s] Waiting for ARuco data..", getCartesianEstimatorLimb().c_str());
    int cnt=0;

    ros::Rate r(10);
    while (!cartest_ok)
    {
        if (cnt!=0) // let's skip the first one since it is very likely to occur
        {
            ROS_WARN("No callback from ARuco. Is ARuco running?");
        }
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("No callback from ARuco! Stopping.");
            return false;
        }

        r.sleep();
    }

    cnt=0;
    while (!object_found)
    {
        if (cnt!=0) // let's skip the first one since it is very likely to occurr
        {
            ROS_WARN("Object with ID %i not found. Is the object there?", getMarkerID());
        }
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("Object with ID %i not found! Stopping.", getMarkerID());
            return false;
        }

        r.sleep();
    }

    return true;
}

CartesianEstimatorClient::~CartesianEstimatorClient()
{

}

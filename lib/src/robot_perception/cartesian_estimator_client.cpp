#include "robot_perception/cartesian_estimator_client.h"

using namespace std;

CartesianEstimatorClient::CartesianEstimatorClient(string _name, string _limb) :
                        nh(_name), limb(_limb), cartest_ok(false),
                        object_found(false), object_name("")
{
    cartest_sub = nh.subscribe("/hsv_detector/objects", SUBSCRIBER_BUFFER,
                                &CartesianEstimatorClient::ObjectCb, this);
}

void CartesianEstimatorClient::clearMarkerPose()
{
    cartest_ok   = false;
    object_found = false;
}

void CartesianEstimatorClient::ObjectCb(const baxter_collaboration::ObjectsArray& _msg)
{
    for (size_t i = 0; i < _msg.objects.size(); ++i)
    {
        // ROS_DEBUG("Processing object with id %i",_msg.objects[i].id);

        if (_msg.objects[i].name == getObjectName())
        {
            curr_object_pos = _msg.objects[i].pose.position;
            curr_object_ori = _msg.objects[i].pose.orientation;

            ROS_DEBUG("Marker is in: %g %g %g", curr_object_pos.x,
                                                curr_object_pos.y,
                                                curr_object_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", curr_object_ori.x,
            //                                       curr_object_ori.y,
            //                                       curr_object_ori.z,
            //                                       curr_object_ori.w);

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
            ROS_WARN("Object with name %s not found. Is the object there?", getObjectName().c_str());
        }
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("Object with name %s not found! Stopping.", getObjectName().c_str());
            return false;
        }

        r.sleep();
    }

    return true;
}

CartesianEstimatorClient::~CartesianEstimatorClient()
{

}

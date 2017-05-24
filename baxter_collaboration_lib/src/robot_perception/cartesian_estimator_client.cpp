#include <algorithm>

#include "robot_perception/cartesian_estimator_client.h"

using namespace std;

CartesianEstimatorClient::CartesianEstimatorClient(string _name, string _limb) :
              rnh(_name), limb(_limb), cartest_ok(false), objects_found(false),
                                          object_found(false), object_name("")
{
    cartest_sub = rnh.subscribe("/hsv_detector/objects", SUBSCRIBER_BUFFER,
                                &CartesianEstimatorClient::ObjectCb, this);
}

void CartesianEstimatorClient::resetCartEst()
{
    cartest_ok   = false;
}

void CartesianEstimatorClient::clearObjFound()
{
    object_found = false;
}

void CartesianEstimatorClient::clearObjsFound()
{
    objects_found = false;

    available_objects.clear();
}

void CartesianEstimatorClient::ObjectCb(const baxter_collaboration_msgs::ObjectsArray& _msg)
{
    // ROS_INFO("ObjectCb");
    if (_msg.objects.size() > 0)
    {
        available_objects.clear();
    }

    for (size_t i = 0; i < _msg.objects.size(); ++i)
    {
        // ROS_DEBUG("Processing object with id %i",_msg.objects[i].id);

        available_objects.push_back(_msg.objects[i].name);
        objects_found = true;

        if (_msg.objects[i].name == getObjectName())
        {
            curr_object_pos = _msg.objects[i].pose.position;
            curr_object_ori = _msg.objects[i].pose.orientation;

            ROS_DEBUG("Object is in: %g %g %g", curr_object_pos.x,
                                                curr_object_pos.y,
                                                curr_object_pos.z);
            // ROS_INFO("Object is in: %g %g %g %g", curr_object_ori.x,
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

std::vector<string> CartesianEstimatorClient::getAvailableObjects(std::vector<string> _objects)
{
    std::vector<string> res;

    for (size_t i = 0; i < _objects.size(); ++i)
    {
        if(std::find(available_objects.begin(), available_objects.end(),
                                _objects[i]) != available_objects.end())
        {
            /* available_objects contains _objects[i] */
            res.push_back(_objects[i]);
        }
    }

    return res;
}

bool CartesianEstimatorClient::waitForCartEstOK()
{
    resetCartEst();

    int cnt=0;
    ros::Rate r(10);

    while (!cartest_ok)
    {
        ROS_WARN_COND(cnt>0, "No callback from CartesianEstimator. Is CartesianEstimator running?");
        ++cnt;

        if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
        {
            ROS_ERROR("No callback from CartesianEstimator! Stopping.");
            return false;
        }

        r.sleep();
    }

    return true;
}

bool CartesianEstimatorClient::waitForCartEstObjsFound()
{
    clearObjsFound();

    int cnt=0;
    ros::Rate r(10);

    while (!objects_found)
    {
        ROS_WARN_COND(cnt>0, "Objects not found. Are there any the objects there?");
        ++cnt;

        if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
        {
            ROS_ERROR("Objects not found! Stopping.");
            return false;
        }

        r.sleep();
    }

    return true;
}

bool CartesianEstimatorClient::waitForCartEstObjFound()
{
    clearObjFound();

    int cnt=0;
    ros::Rate r(10);

    while (!object_found)
    {
        ROS_WARN_COND(cnt>0, "Object with name %s not found. Is the object there?", getObjectName().c_str());
        ++cnt;

        if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
        {
            ROS_ERROR("Object with name %s not found! Stopping.", getObjectName().c_str());
            return false;
        }

        r.sleep();
    }

    return true;
}

bool CartesianEstimatorClient::waitForCartEstData()
{
    ROS_INFO("[%s] Waiting for CartesianEstimator data..", getCartEstLimb().c_str());

    if (!waitForCartEstOK())        return false;
    if (!waitForCartEstObjsFound()) return false;
    if (!waitForCartEstObjFound())  return false;

    return true;
}

CartesianEstimatorClient::~CartesianEstimatorClient()
{

}

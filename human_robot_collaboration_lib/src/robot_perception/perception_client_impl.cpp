#include "robot_perception/perception_client_impl.h"

using namespace std;

ARucoClient::ARucoClient(string _name, string _limb) :
                         ClientTemplate(_name, _limb)
{
    sub = ctnh.subscribe("/markers/"+getClientLimb(), SUBSCRIBER_BUFFER,
                         &ARucoClient::ObjectCb, this);

    object_id = -1;
}

void ARucoClient::ObjectCb(const aruco_msgs::MarkerArray& _msg)
{
    ROS_INFO_COND(ct_print_level>=4, "ObjectCb");

    if (_msg.markers.size() > 0)
    {
        available_objects.clear();
    }

    for (size_t i = 0; i < _msg.markers.size(); ++i)
    {
        // ROS_DEBUG("Processing object with id %i",_msg.markers[i].id);

        available_objects.push_back(int(_msg.markers[i].id));
        objects_found = true;

        if (int(_msg.markers[i].id) == getObjectID())
        {
            curr_object_pos = _msg.markers[i].pose.pose.position;
            curr_object_ori = _msg.markers[i].pose.pose.orientation;

            ROS_DEBUG("Object is in: %g %g %g", curr_object_pos.x,
                                                curr_object_pos.y,
                                                curr_object_pos.z);
            // ROS_INFO("Object is in: %g %g %g %g", curr_object_ori.x,
            //                                       curr_object_ori.y,
            //                                       curr_object_ori.z,
            //                                       curr_object_ori.w);

            if (!object_found) { object_found = true; }
        }
    }

    if (not is_ok) { is_ok = true; }
}

ARucoClient::~ARucoClient()
{

}

CartesianEstimatorClient::CartesianEstimatorClient(string _name, string _limb) :
                                                   ClientTemplate(_name, _limb)
{
    sub = ctnh.subscribe("/objects/"+getClientLimb(), SUBSCRIBER_BUFFER,
                         &CartesianEstimatorClient::ObjectCb, this);

    object_id = "";
}

void CartesianEstimatorClient::ObjectCb(const human_robot_collaboration_msgs::ObjectsArray& _msg)
{
    ROS_INFO_COND(ct_print_level>=4, "ObjectCb");

    if (_msg.objects.size() > 0)
    {
        available_objects.clear();
    }

    for (size_t i = 0; i < _msg.objects.size(); ++i)
    {
        // ROS_DEBUG("Processing object with id %i",_msg.objects[i].id);

        available_objects.push_back(_msg.objects[i].name);
        objects_found = true;

        if (_msg.objects[i].name == getObjectID())
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

            if (!object_found) { object_found = true; }
        }
    }

    if (not is_ok) { is_ok = true; }
}

CartesianEstimatorClient::~CartesianEstimatorClient()
{

}

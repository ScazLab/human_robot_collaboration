#include <algorithm>

#include "robot_perception/aruco_client.h"

using namespace std;

ARucoClient::ARucoClient(string name, string limb) : _nh(name), _limb(limb),
                                      aruco_ok(false), markers_found(false),
                                         marker_found(false), marker_id(-1)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers",
                               SUBSCRIBER_BUFFER, &ARucoClient::ARucoCb, this);
}

void ARucoClient::resetARuco()
{
    aruco_ok   = false;
}

void ARucoClient::clearMarkerFound()
{
    marker_found = false;
}

void ARucoClient::clearMarkersFound()
{
    markers_found = false;

    available_markers.clear();
}

void ARucoClient::ARucoCb(const aruco_msgs::MarkerArray& msg)
{
    if (msg.markers.size() > 0)
    {
        available_markers.clear();
    }

    for (size_t i = 0; i < msg.markers.size(); ++i)
    {
        // ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        available_markers.push_back(int(msg.markers[i].id));
        markers_found = true;

        if (int(msg.markers[i].id) == getMarkerID())
        {
            curr_marker_pos = msg.markers[i].pose.pose.position;
            curr_marker_ori = msg.markers[i].pose.pose.orientation;

            ROS_DEBUG("Marker is in: %g %g %g", curr_marker_pos.x,
                                                curr_marker_pos.y,
                                                curr_marker_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", curr_marker_ori.x,
            //                                       curr_marker_ori.y,
            //                                       curr_marker_ori.z,
            //                                       curr_marker_ori.w);

            if (!marker_found)
            {
                marker_found = true;
            }
        }
    }

    if (!aruco_ok)
    {
        aruco_ok = true;
    }
}

std::vector<int> ARucoClient::getAvailableMarkers(std::vector<int> _markers)
{
    std::vector<int> res;

    for (size_t i = 0; i < _markers.size(); ++i)
    {
        if(std::find(available_markers.begin(), available_markers.end(),
                                _markers[i]) != available_markers.end())
        {
            /* available_markers contains _markers[i] */
            res.push_back(_markers[i]);
        }
    }

    return res;
}

bool ARucoClient::waitForARucoOK()
{
    resetARuco();

    int cnt = 0;
    ros::Rate r(10);

    while (!aruco_ok)
    {
        ROS_WARN_COND(cnt>0, "No callback from ARuco. Is ARuco running?");
        ++cnt;

        if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
        {
            ROS_ERROR("No callback from ARuco! Stopping.");
            return false;
        }

        r.sleep();
    }

    return true;
}

bool ARucoClient::waitForARucoMarkersFound()
{
    clearMarkersFound();

    int cnt=0;
    ros::Rate r(10);

    while (!markers_found)
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

bool ARucoClient::waitForARucoMarkerFound()
{
    clearMarkerFound();

    int cnt=0;
    ros::Rate r(10);

    while (!marker_found)
    {
        ROS_WARN_COND(cnt>0, "Object with ID %i not found. Is the object there?", getMarkerID());
        ++cnt;

        if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
        {
            ROS_ERROR("Object with ID %i not found! Stopping.", getMarkerID());
            return false;
        }

        r.sleep();
    }

    return true;
}

bool ARucoClient::waitForARucoData()
{
    ROS_INFO("[%s] Waiting for ARuco data..", getArucoLimb().c_str());

    if (!waitForARucoOK())           return false;
    if (!waitForARucoMarkersFound()) return false;
    if (!waitForARucoMarkerFound())  return false;

    return true;
}

ARucoClient::~ARucoClient()
{

}

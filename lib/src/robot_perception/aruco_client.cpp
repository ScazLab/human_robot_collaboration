#include "robot_perception/aruco_client.h"

using namespace std;

ARucoClient::ARucoClient(string name, string limb) :
                        _nh(name), _limb(limb), aruco_ok(false),
                        marker_id(-1), marker_found(false)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers",
                               SUBSCRIBER_BUFFER, &ARucoClient::ARucoCb, this);
}

void ARucoClient::clearMarkerPose()
{
    aruco_ok     = false;
    marker_found = false;
}

void ARucoClient::ARucoCb(const aruco_msgs::MarkerArray& msg)
{
    for (int i = 0; i < msg.markers.size(); ++i)
    {
        // ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        if (msg.markers[i].id == getMarkerID())
        {
            _curr_marker_pos = msg.markers[i].pose.pose.position;
            _curr_marker_ori = msg.markers[i].pose.pose.orientation;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_marker_pos.x,
                                                _curr_marker_pos.y,
                                                _curr_marker_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", _curr_marker_ori.x,
            //                                       _curr_marker_ori.y,
            //                                       _curr_marker_ori.z,
            //                                       _curr_marker_ori.w);

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

bool ARucoClient::waitForARucoData()
{
    clearMarkerPose();
    ROS_INFO("[%s] Waiting for ARuco data..", getArucoLimb().c_str());
    int cnt=0;

    ros::Rate r(10);
    while (!aruco_ok)
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
    while (!marker_found)
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

ARucoClient::~ARucoClient()
{

}

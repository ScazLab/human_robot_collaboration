#ifndef __ARUCO_CLIENT__
#define __ARUCO_CLIENT__

#include <ros/ros.h>
#include <ros/console.h>

#include <aruco_msgs/MarkerArray.h>

#include "robot_utils/utils.h"

class ARucoClient
{
private:
    ros::NodeHandle _nh;

    std::string _limb; // Limb of the gripper: left or right

    // Subscriber to the ARuco detector,
    ros::Subscriber _aruco_sub;

    // List of available markers
    std::vector<int> available_markers;

    // Bool to check if ARuco is fine or not
    bool     aruco_ok;

    // Bool to check if the marker was found or not
    bool marker_found;

    // ID of the marker to detect
    int     marker_id;

    // Marker position and orientation
    geometry_msgs::Point        _curr_marker_pos;
    geometry_msgs::Quaternion   _curr_marker_ori;

protected:

    /**
     * Clears the marker pose to reset its state internally
     */
    void clearMarkerPose();

    /**
     * Callback function for the ARuco topic
     * @param msg the topic message
     */
    void ARucoCb(const aruco_msgs::MarkerArray& msg);

    /**
     * Waits to get feedback from aruco
     * @return true/false if success/failure
     */
    bool waitForARucoOK();

    /**
     * Waits for useful data coming from ARuco
     * @return true/false if success/failure
     */
    bool waitForARucoData();

    /*
     * Check availability of the ARuco data
    */
    bool is_aruco_ok() { return aruco_ok; };

    /* SETTERS */
    void setMarkerID(int _id)            { marker_id =     _id; };

    /* GETTERS */
    geometry_msgs::Point      getMarkerPos() { return _curr_marker_pos; };
    geometry_msgs::Quaternion getMarkerOri() { return _curr_marker_ori; };

    std::string getArucoLimb() { return     _limb; };
    int         getMarkerID()  { return marker_id; };

    /**
     * Returns a list of available markers
     * @return a list of available markers
     */
    std::vector<int> getAvailableMarkers() { return available_markers; };

    /**
     * Looks if a set of markers is present among those available.
     * @return the subset of available markers among those avilable
     */
    std::vector<int> getAvailableMarkers(std::vector<int> _markers);

public:
    ARucoClient(std::string name, std::string limb);
    ~ARucoClient();

};

#endif

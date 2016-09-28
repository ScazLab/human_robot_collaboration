#ifndef __ARUCO_CLIENT__
#define __ARUCO_CLIENT__

#include <ros/ros.h>
#include <ros/console.h>

#include <aruco_msgs/MarkerArray.h>

#include "robot_utils/utils.h"

class ARucoClient
{
private:
    std::string _limb; // Limb of the gripper: left or right

    // Subscriber to the ARuco detector,
    // plus some flags related to it
    ros::Subscriber _aruco_sub;
    bool              aruco_ok;
    bool          marker_found;

    int          marker_id;

    // Marker position and orientation
    geometry_msgs::Point        _curr_marker_pos;
    geometry_msgs::Quaternion   _curr_marker_ori;

    ros::NodeHandle _nh;

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
     * Waits for useful data coming from ARuco
     * @return true/false if success/failure
     */
    bool waitForARucoData();

    /*
     * Check availability of the ARuco data
    */
    bool is_aruco_ok() { return aruco_ok; };

    /* Self-explaining "setters" */
    void setMarkerID(int _id)            { marker_id =     _id; };

    /* Self-explaining "getters" */
    geometry_msgs::Point      getMarkerPos() { return _curr_marker_pos; };
    geometry_msgs::Quaternion getMarkerOri() { return _curr_marker_ori; };

    std::string getArucoLimb() { return     _limb; };
    int         getMarkerID()  { return marker_id; };

public:
    ARucoClient(std::string name, std::string limb);
    ~ARucoClient();

};

#endif

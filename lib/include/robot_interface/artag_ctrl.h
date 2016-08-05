#ifndef __ARTAG_CTRL_H__
#define __ARTAG_CTRL_H__

#include <aruco_msgs/MarkerArray.h>

#include "robot_interface/arm_ctrl.h"

class ARTagCtrl : public ArmCtrl
{
private:
    double elap_time;

    ros::Subscriber _aruco_sub;
    bool              aruco_ok;
    geometry_msgs::Point        _curr_marker_pos;
    geometry_msgs::Quaternion   _curr_marker_ori;

    void clearMarkerPose();

    void ARucoCb(const aruco_msgs::MarkerArray& msg);

    bool waitForARucoData();

    bool hoverAbovePool();

    bool moveObjectTowardHuman();

    bool pickARTag();

    bool pickObject();

    bool passObject();

    bool prepare4HandOver();

    bool handOver();

    bool waitForOtherArm(double _wait_time = 60.0, bool disable_coll_av = false);

    /**
     * Computes the end-effector orientation needed to pick the object up with a constant
     * orientation. Needed by the hand-over action since it requires the object to be picked
     * up consistently.
     * @return the desired end-effector orientation, expressed in quaternion form
     */
    geometry_msgs::Quaternion computeHOorientation();

    /*
     * Check availability of the aruco data
    */
    bool is_aruco_ok() { return aruco_ok; };

protected:
    bool doAction(int s, std::string a);

public:
    ARTagCtrl(std::string _name, std::string _limb);

    ~ARTagCtrl();
};

#endif

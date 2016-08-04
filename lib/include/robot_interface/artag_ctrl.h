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

    /*
     * Moves arm to the requested pose. This differs from ROSThread::goToPose because it 
     * does not check if the final pose has been reached, but rather it goes in open-loop
     * unitil a fisical contact with the table is reached
     * 
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow);

    void ARucoCb(const aruco_msgs::MarkerArray& msg);

    bool waitForARucoData();

    bool hoverAbovePool();

    bool moveObjectTowardHuman();

    bool pickARTag();

    bool pickObject();

    bool passObject();

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

    /**
     * Callback for the service that lets the two limbs interact
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                            baxter_collaboration::AskFeedback::Response &res);

    ~ARTagCtrl();
};

#endif

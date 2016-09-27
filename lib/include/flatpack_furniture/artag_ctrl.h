#ifndef __ARTAG_CTRL_H__
#define __ARTAG_CTRL_H__

#include <aruco_msgs/MarkerArray.h>

#include "robot_interface/arm_ctrl.h"

class ARTagCtrl : public ArmCtrl
{
private:
    double elap_time;

    // Subscriber to the ARuco detector,
    // plus some flags related to it
    ros::Subscriber _aruco_sub;
    bool              aruco_ok;
    bool          marker_found;

    // Marker position and orientation
    geometry_msgs::Point        _curr_marker_pos;
    geometry_msgs::Quaternion   _curr_marker_ori;

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

    /**
     * [hoverAbovePool description]
     * @return true/false if success/failure
     */
    bool hoverAbovePool();

    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [pickARTag description]
     * @return true/false if success/failure
     */
    bool pickARTag();

    /**
     * [getObject description]
     * @return true/false if success/failure
     */
    bool getObject();

    /**
     * [recoverRelease description]
     * @return true/false if success/failure
     */
    bool recoverGet();

    /**
     * [recoverRelease description]
     * @return true/false if success/failure
     */
    bool recoverRelease();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * [prepare4HandOver description]
     * @return true/false if success/failure
     */
    bool prepare4HandOver();

    /**
     * [handOver description]
     * @return true/false if success/failure
     */
    bool handOver();

    /**
     * [waitForOtherArm description]
     * @param  _wait_time      time to wait
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool waitForOtherArm(double _wait_time = 60.0, bool disable_coll_av = false);

    /**
     * [hoverAboveTableStrict description]
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool hoverAboveTableStrict(bool disable_coll_av = false);

    /**
     * Computes the end-effector orientation needed to pick the object up with a constant
     * orientation. Needed by the hand-over action since it requires the object to be picked
     * up consistently.
     * @return the desired end-effector orientation, expressed in quaternion form
     */
    geometry_msgs::Quaternion computeHOorientation();

    /*
     * Check availability of the ARuco data
    */
    bool is_aruco_ok() { return aruco_ok; };

protected:
    /**
     * Executes the arm-specific and task-specific actions.
     *
     * @param  s the state of the system before starting the action
     * @param  a the action to do
     * @return   true/false if success/failure
     */
    bool doAction(int s, std::string a);

public:
    /**
     * Constructor
     */
    ARTagCtrl(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~ARTagCtrl();
};

#endif

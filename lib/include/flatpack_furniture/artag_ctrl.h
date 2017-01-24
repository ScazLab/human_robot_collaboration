#ifndef __ARTAG_CTRL_H__
#define __ARTAG_CTRL_H__

#include <stdlib.h>
#include <time.h>

#include <aruco_msgs/MarkerArray.h>

#include "robot_interface/arm_ctrl.h"
#include "robot_perception/aruco_client.h"

class ARTagCtrl : public ArmCtrl, public ARucoClient
{
private:
    // Elapsed time (useful for testing purposes)
    double elap_time;

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
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * [getPassObject description]
     * @return true/false if success/failure
     */
    bool getPassObject();

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
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

    /**
     * Computes the end-effector orientation needed to pick the object up with a constant
     * orientation. Needed by the hand-over action since it requires the object to be picked
     * up consistently.
     * @return the desired end-effector orientation, expressed in quaternion form
     */
    geometry_msgs::Quaternion computeHOorientation();

protected:
    /**
     * Chooses the object to act upon according to some rule. This method
     * needs to be specialized in any derived class because it is dependent
     * on the type of action and the type of sensory capabilities available.
     *
     * @param _objs The list of IDs of objects to choose from
     * @return      the ID of the chosen object (by default the ID of the
     *              first object will be chosen)
     */
    int chooseObjectID(std::vector<int> _objs);

public:
    /**
     * Constructor
     */
    ARTagCtrl(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~ARTagCtrl();

    void setObjectID(int _obj);
};

#endif

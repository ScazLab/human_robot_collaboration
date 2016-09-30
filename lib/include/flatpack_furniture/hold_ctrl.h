#ifndef __HOLD_CTRL_H__
#define __HOLD_CTRL_H__

#include "robot_interface/arm_ctrl.h"

class HoldCtrl : public ArmCtrl
{
private:
    /**
     * [goHoldPose description]
     * @param  height [description]
     * @return        true/false if success/failure
     */
    bool goHoldPose(double height);

    /**
     * [startHold description]
     * @return true/false if success/failure
     */
    bool startHold();

    /**
     * [endHold description]
     * @return true/false if success/failure
     */
    bool endHold();

    /**
     * [holdObject description]
     * @return true/false if success/failure
     */
    bool holdObject();

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

public:
    /**
     * Constructor
     */
    HoldCtrl(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Callback for the service that lets the two limbs interact
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                            baxter_collaboration::AskFeedback::Response &res);

    /**
     * Destructor
     */
    ~HoldCtrl();
};

#endif

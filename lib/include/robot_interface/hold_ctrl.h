#ifndef __HOLD_CTRL_H__
#define __HOLD_CTRL_H__

#include "robot_interface/arm_ctrl.h"

class HoldCtrl : public ArmCtrl
{
private:

    bool goHoldPose(double height);

    bool holdObject();

    bool handOver();

    bool prepare4HandOver();

    bool waitForOtherArm(double _wait_time = 60.0, bool disable_coll_av = false);

    bool hoverAboveTableStrict(bool disable_coll_av = false);

protected:
    bool doAction(int s, std::string a);

public:
    HoldCtrl(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Callback for the service that lets the two limbs interact
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                            baxter_collaboration::AskFeedback::Response &res);

    ~HoldCtrl();
};

#endif

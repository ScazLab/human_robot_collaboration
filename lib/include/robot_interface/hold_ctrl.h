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

protected:
    bool doAction(int s, std::string a);

public:
    HoldCtrl(std::string _name, std::string _limb);
        
    ~HoldCtrl();
};

#endif

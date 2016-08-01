#ifndef __HOLD_CONTROLLER_H__
#define __HOLD_CONTROLLER_H__

#include "robot_interface/ros_thread.h"
#include "robot_interface/gripper.h"

class HoldController : public ROSThread, public Gripper
{
private:
    std::string action;

    bool goHoldPose(double height);

    bool hoverAboveTable(double height);

    void recoverFromError();

    bool holdObject();

    bool goHome();

protected:

    void InternalThreadEntry();

public:
    HoldController(std::string limb);

    void setAction(std::string _action) { action = _action; };
        
    ~HoldController();
};

#endif

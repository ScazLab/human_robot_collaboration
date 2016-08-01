#ifndef __HOLD_CONTROLLER_H__
#define __HOLD_CONTROLLER_H__

#include "robot_interface/ros_thread.h"

class HoldController : public ROSThread
{
private:
    std::string action;

    bool goHoldPose(double height);

    bool hoverAboveTable(double height);

    void recoverFromError();

    bool holdObject();

    bool goHome();

    bool releaseObject();

protected:

    void InternalThreadEntry();

public:
    HoldController(std::string limb);

    void setAction(std::string _action) { action = _action; };
        
    ~HoldController();
};

#endif

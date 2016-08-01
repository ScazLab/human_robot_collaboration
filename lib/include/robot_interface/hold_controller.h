#ifndef HOLD_CONTROLLER_H
#define HOLD_CONTROLLER_H

#include "arm_controller.h"

class HoldController : public ROSThread
{
private:
    std::string action;

    bool goHoldPose(double height);

    bool hoverAboveTable(double height);

    bool holdObject();

    bool goHome();

    bool releaseObject();

protected:

    void InternalThreadEntry();

public:
    HoldController(std::string limb);

    void setAction(std::string _action) { action = _action; };

    void setMarkerID(int _id) { marker_id = _id; };
        
    ~HoldController();
};

#endif

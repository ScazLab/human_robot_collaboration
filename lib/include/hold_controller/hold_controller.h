#ifndef HOLD_CONTROLLER_H
#define HOLD_CONTROLLER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_control/arm_controller.h>
#include <artag_controller/artag_controller.h>

#define HOME_POSITION_RIGHT_ARM 0.60, -0.30, 0.150

#define HORIZONTAL_ORIENTATION_RIGHT_ARM    -0.175730746765, 0.67317042445, 0.1902242414, 0.692657940308
#define VERTICAL_ORIENTATION_RIGHT_ARM     -0.14007673309, 0.989696832226, -0.0103020489473, 0.0277949080985

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
        
    ~HoldController();
};

#endif

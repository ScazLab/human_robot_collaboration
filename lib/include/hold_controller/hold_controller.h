#ifndef HOLD_CONTROLLER_H
#define HOLD_CONTROLLER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_controller/arm_controller.h>

// #include <baxterCollaboration/artag_controller.h>

#define ACTION_HOLD    "hold"

#define HEIGHT  0.32
#define PX      0.80
#define PY      -0.26
#define HORIZONTAL_ORIENTATION_LEFT_ARM     -0.0155113251266, 0.703354199922, -0.102825501874, 0.703192139041
#define HORIZONTAL_ORIENTATION_RIGHT_ARM     -0.253427232811, 0.683506109838, 0.226046021433, 0.646140256971


class HoldController : public ROSThread
{
private:
    double elapsed_time;

    geometry_msgs::Pose _curr_marker_pose;
    ros::Publisher _gripper_pub;

    bool goHoldPose(double height);
    bool waitForButton();

protected:

void InternalThreadEntry();

public:
    HoldController(std::string limb);

    void actionHold();
        
    ~HoldController();
};

#endif

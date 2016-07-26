#include "hold_controller/hold_controller.h"

using namespace std;

HoldController::HoldController(std::string limb) : ROSThread(limb)
{
    elapsed_time = 0; // TODO keep?

    // _button_sub //TODO
}

HoldController::~HoldController() {}

void HoldController::InternalThreadEntry()
{
    if (int(getState()) == START)
    {
        bool res = goHoldPose(HEIGHT);
        if (res)
        {
            ros::Rate(100).sleep();
            ros::spinOnce();
            res = waitForForceInteraction();
        }
        if (res)
        {
            _gripper->suck();
            res = waitForButton();
        }
        if (res)
        {
            _gripper->blow();
            setState(PASSED);
        } else
        {
            setState(ERROR);
        }
    }
    else
    {
        ROS_ERROR("Invalid State %i", int(getState()));
        setState(ERROR);
    }

    pthread_exit(NULL);
    return;
}

bool HoldController::goHoldPose(double height)
{
    bool res;

    if (ROSThread::getLimb() == "left")
    {
      res = goToPose(PX, PY, height, HORIZONTAL_ORIENTATION_LEFT_ARM);
    }
    else
    {
      res = goToPose(PX, PY, height, HORIZONTAL_ORIENTATION_RIGHT_ARM);
    }

    return res;
}

void HoldController::actionHold()
{
    setState(START);
}


bool HoldController::waitForButton() {
    // TODO: implement
    ros::Duration(2.0).sleep();
    return true;
}

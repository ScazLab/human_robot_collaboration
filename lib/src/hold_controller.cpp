#include "hold_controller/hold_controller.h"

using namespace std;

HoldController::HoldController(std::string limb) : ROSThread(limb)
{

}

HoldController::~HoldController() {}

void HoldController::InternalThreadEntry()
{
    if (int(getState()) == START || int(getState()) == PASSED)
    {
        if (holdObject())   setState(PASSED);
        else
        {
            releaseObject();
            HoldController::hoverAboveTable(POS_LOW);
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

bool HoldController::holdObject()
{
    if (!goHoldPose(0.24))              return false;
    if (!waitForForceInteraction())     return false;
    if (!suckObject())                  return false;
    ros::Duration(2.0).sleep();
    if (!waitForForceInteraction(30.0)) return false;
    if (!releaseObject())               return false;
    if (!HoldController::hoverAboveTable(POS_LOW))               return false;

    return true;
}

bool HoldController::goHoldPose(double height)
{
    return ROSThread::goToPose(0.80, -0.4, height, HORIZONTAL_ORIENTATION_RIGHT_ARM);
}

bool HoldController::hoverAboveTable(double height)
{
    return ROSThread::goToPose(HOME_POSITION_RIGHT_ARM, VERTICAL_ORIENTATION_RIGHT_ARM);
}

bool HoldController::goHome()
{
    bool res = releaseObject();
    res = res && HoldController::hoverAboveTable(POS_LOW);
    setState(START);
    return res;
}

bool HoldController::releaseObject()
{
    bool res = ROSThread::releaseObject();
    setState(START);
    return res;
}


bool HoldController::waitForButton()
{
    // TODO: implement
    ros::Duration(3.0).sleep();
    return true;
}

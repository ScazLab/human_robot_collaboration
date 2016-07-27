#include "robot_interface/hold_controller.h"

using namespace std;

HoldController::HoldController(std::string limb) : ROSThread(limb)
{

}

HoldController::~HoldController() {}

void HoldController::InternalThreadEntry()
{
    if (action == ACTION_HOME)
    {
        setState(WORKING);
        if (goHome())   setState(START);
        else            setState(ERROR);
    }
    else if (action == ACTION_RELEASE)
    {
        setState(WORKING);
        if (releaseObject())   setState(START);
        else                   setState(ERROR);
    }
    else if (action == ACTION_HOLD && 
             (int(getState()) == START || int(getState()) == PASSED))
    {
        setState(WORKING);
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
    if (!gripObject())                  return false;
    ros::Duration(2.0).sleep();
    if (!waitForForceInteraction(30.0)) return false;
    if (!releaseObject())               return false;
    if (!goHome())                      return false;

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
    return HoldController::hoverAboveTable(POS_LOW);
}

bool HoldController::releaseObject()
{
    return ROSThread::releaseObject();
}

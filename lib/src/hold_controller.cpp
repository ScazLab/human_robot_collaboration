#include "robot_interface/hold_controller.h"

using namespace std;

HoldController::HoldController(std::string limb) : ROSThread(limb),
                                                   Gripper(limb)
{
    setState(START);
}

HoldController::~HoldController() {}

void HoldController::InternalThreadEntry()
{
    int s = int(getState());

    setState(WORKING);

    if (action == ACTION_HOME)
    {
        if (goHome())   setState(START);
        else            setState(ERROR);
    }
    else if (action == ACTION_RELEASE)
    {
        if (releaseObject())   setState(START);
        else                   setState(ERROR);
    }
    else if (action == ACTION_HOLD && (s == START ||
                                       s == ERROR ||
                                       s == PASSED))
    {
        if (holdObject())   setState(PASSED);
        else                recoverFromError();
    }
    else
    {
        ROS_ERROR("Invalid State %i", s);
        setState(ERROR);
    }

    pthread_exit(NULL);
    return;
}

void HoldController::recoverFromError()
{
    releaseObject();
    goHome();
    setState(ERROR);
}

bool HoldController::holdObject()
{
    if (!goHoldPose(0.24))              return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!gripObject())                  return false;
    ros::Duration(2.0).sleep();
    if (!waitForForceInteraction(30.0)) return false;
    if (!releaseObject())               return false;
    ros::Duration(2.0).sleep();
    if (!goHome())                      return false;

    return true;
}

bool HoldController::goHoldPose(double height)
{
    return ROSThread::goToPose(0.80, -0.4, height,
                               HORIZONTAL_ORIENTATION_RIGHT_ARM);
}

bool HoldController::hoverAboveTable(double height)
{
    return ROSThread::goToPose(HOME_POSITION_RIGHT_ARM, height,
                               VERTICAL_ORIENTATION_RIGHT_ARM);
}

bool HoldController::goHome()
{
    return HoldController::hoverAboveTable(POS_LOW);
}

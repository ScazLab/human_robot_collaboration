#include "robot_interface/hold_ctrl.h"

using namespace std;

HoldCtrl::HoldCtrl(std::string _name, std::string _limb) : 
                                    ArmCtrl(_name,_limb)
{
    if (!goHome()) setState(ERROR);
}

bool HoldCtrl::doAction(int s, std::string a)
{
    if (a == ACTION_HOLD && (s == START ||
                                  s == ERROR ||
                                  s == DONE  ))
    {
        if (holdObject())
        {
            setState(DONE);
            return true;
        }   
        else recoverFromError();
    }
    else
    {
        ROS_ERROR("[%s] Invalid State %i", getLimb().c_str(), s);
    }

    return false;
}

bool HoldCtrl::holdObject()
{
    if (!goHoldPose(0.24))                return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction(30.0))   return false;
    if (!gripObject())                    return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction(180.0))  return false;
    if (!releaseObject())                 return false;
    ros::Duration(1.0).sleep();
    if (!goHome())                         return false;

    return true;
}

bool HoldCtrl::goHoldPose(double height)
{
    return ROSThread::goToPose(0.80, -0.4, height,
                               HORIZONTAL_ORI_R);
}

HoldCtrl::~HoldCtrl()
{

}

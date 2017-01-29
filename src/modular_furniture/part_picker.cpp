#include "part_picker.h"

using namespace std;

PartPicker::PartPicker(std::string _name, std::string _limb, bool _no_robot) :
                       ARTagCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();

    removeAction(ACTION_PASS);
    insertAction(ACTION_PASS, static_cast<f_action>(&PartPicker::passObject));

    printActionDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

void PartPicker::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

bool PartPicker::passObject()
{
    if (getObjectID() == 200)
    {
        return ARTagCtrl::passObject();
    }

    if (getPrevAction() != ACTION_GET)  return false;
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForUserFb())               return false;
    if (!goToPose(0.50, 0.93, 0.2, POOL_ORI_L)) return false;
    ros::Duration(0.25).sleep();
    if (!releaseObject())               return false;
    if (!homePoseStrict())              return false;

    return true;
}

bool PartPicker::waitForUserFb(double _wait_time)
{
    ROS_INFO("[%s] Waiting user feedback for %g [s]",
                      getLimb().c_str(), _wait_time);

    cuff_button_pressed = false;

    ros::Time _init = ros::Time::now();

    ros::Rate(THREAD_FREQ);
    while(RobotInterface::ok())
    {
        if (cuff_button_pressed == true)        return true;

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No user feedback has been detected in %g [s]!",_wait_time);
            return false;
        }
    }

    return false;
}

void PartPicker::cuffUpperCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        cuff_button_pressed = true;
    }

    return;
}

PartPicker::~PartPicker()
{

}

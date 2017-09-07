#include "part_picker.h"

using namespace std;

PartPicker::PartPicker(std::string _name, std::string _limb, bool _use_robot) :
                       ARTagCtrl(_name, _limb, _use_robot)
{
    setHomeConfiguration();

    removeAction(ACTION_PASS);
    insertAction(ACTION_PASS, static_cast<f_action>(&PartPicker::passObject));

    printActionDB();

    if (not _use_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

void PartPicker::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

bool PartPicker::passObject()
{
    if (getPrevAction() != ACTION_GET)  return false;
    if (!moveObjectTowardHuman())       return false;
    if (!waitForUserCuffUpperFb())      return false;

    if (ClientTemplate<int>::getObjectID() != 200)
    {
        if (!goToPose(0.50, 0.93, 0.2, POOL_ORI_L)) return false;
        ros::Duration(0.25).sleep();
    }

    if (!open())                        return false;
    if (!homePoseStrict())              return false;

    return true;
}

PartPicker::~PartPicker()
{

}

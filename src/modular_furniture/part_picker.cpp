#include "part_picker.h"

using namespace std;

PartPicker::PartPicker(std::string _name, std::string _limb, bool _no_robot) :
                       ARTagCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();

    removeAction(ACTION_HAND_OVER);
    removeAction("recover_"+string(ACTION_RELEASE));
    removeAction("recover_"+string(ACTION_GET));
    removeAction("recover_"+string(ACTION_PASS));
    removeAction("recover_"+string(ACTION_HAND_OVER));

    printActionDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

void PartPicker::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

PartPicker::~PartPicker()
{

}

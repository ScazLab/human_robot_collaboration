#include "tower_building/cube_picker.h"

using namespace std;
using namespace baxter_core_msgs;

CubePicker::CubePicker(std::string _name, std::string _limb, bool _no_robot) :
                       ARucoClient(_name, _limb), ArmCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();

    setState(START);

    printDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool CubePicker::doAction(int s, std::string a)
{
    if (a == ACTION_GET || a == ACTION_PASS)
    {
        if (callAction(a))         return true;
        else                recoverFromError();
    }
    else
    {
        ROS_ERROR("[%s] Invalid Action %s in state %i", getLimb().c_str(), a.c_str(), s);
    }

    return false;
}

void CubePicker::setHomeConfiguration()
{
    setHomeConf(0.7060, -1.2717, 0.3846,  1.5405,
                        -0.1273, 1.3135,  0.3206);
}

CubePicker::~CubePicker()
{

}

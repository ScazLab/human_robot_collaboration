#include "cube_picker.h"

using namespace std;

CubePicker::CubePicker(std::string _name, std::string _limb, bool _use_robot) :
                       ARTagCtrl(_name, _limb, _use_robot)
{
    setHomeConfiguration();

    setState(START);

    insertAction("recover_"+string(ACTION_GET_PASS),
                 static_cast<f_action>(&CubePicker::recoverPickPass));

    printActionDB();

    if (not _use_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool CubePicker::recoverPickPass()
{
    if (!homePoseStrict()) return false;

    if (getPrevAction() == ACTION_GET)
    {
        if (!moveArm("left", 0.1)) return false;
        if (!moveArm("down", 0.3)) return false;
        if (!open())               return false;
        if (!homePoseStrict()) return false;
    }
    return true;
}

void CubePicker::recoverFromError()
{
    if (getInternalRecovery() == true)
    {
        setState(RECOVER);
        recoverPickPass();
        setState(ERROR);
    }
}

bool CubePicker::moveObjectTowardHuman()
{
    ROS_INFO("[%s] Moving object toward human..", getLimb().c_str());
    return goToPose(0.80, 0.26, 0.32, VERTICAL_ORI_L);
}

CubePicker::~CubePicker()
{

}

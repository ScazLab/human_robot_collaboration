#include "tool_picker.h"

using namespace std;
using namespace baxter_core_msgs;

ToolPicker::ToolPicker(std::string _name, std::string _limb, bool _no_robot) :
                       ArmCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));

    printActionDB();

    insertObject(1, "screwdriver");

    printObjectDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ToolPicker::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();
    // if (!pickARTag())               return false;
    if (!gripObject())              return false;
    if (!moveArm("up", 0.3))        return false;
    if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool ToolPicker::passObject()
{
    if (getSubState() != ACTION_GET)    return false;
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!releaseObject())               return false;
    if (!homePoseStrict())              return false;

    return true;
}

bool ToolPicker::getPassObject()
{
    if (!getObject())      return false;
    if (!passObject())      return false;

    return true;
}

bool ToolPicker::moveObjectTowardHuman()
{
    ROS_INFO("[%s] Moving object toward human..", getLimb().c_str());
    return goToPose(0.80, 0.26, 0.32, VERTICAL_ORI_L);
}

void ToolPicker::setHomeConfiguration()
{
    setHomeConf(0.7060, -1.2717, 0.3846,  1.5405,
                        -0.1273, 1.3135,  0.3206);
}

ToolPicker::~ToolPicker()
{

}

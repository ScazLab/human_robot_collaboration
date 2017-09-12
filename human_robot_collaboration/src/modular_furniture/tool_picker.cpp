#include "tool_picker.h"

using namespace              std;
using namespace baxter_core_msgs;

ToolPicker::ToolPicker(string _name, string _limb, bool _use_robot) :
                       ArmPerceptionCtrl(_name, _limb, _use_robot)
{
    setHomeConfiguration();
    setArmSpeed(getArmSpeed() / 1.3);

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));
    insertAction(ACTION_CLEANUP,   static_cast<f_action>(&ToolPicker::cleanUpObject));

    removeAction(ACTION_HOLD);

    insertAction(string(ACTION_HOLD) + "_leg", static_cast<f_action>(&ToolPicker::holdObject));
    insertAction(string(ACTION_HOLD) + "_top", static_cast<f_action>(&ToolPicker::holdObject));

    printActionDB();

    if (not _use_robot) return;

    // reduceSquish();

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ToolPicker::moveObjectToPassPosition(bool &_human)
{
    if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box"  ||
        getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
    {
        if (!hoverAboveTable(Z_LOW))    return false;

        if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
        {
            if (!goToPose(0.63, -0.10, -0.14, VERTICAL_ORI_R2)) return false;
        }
        else
        {
            if (!goToPose(0.63, -0.30, -0.14, VERTICAL_ORI_R2)) return false;
        }
    }
    else
    {
        return ArmCtrl::moveObjectToPassPosition(_human);
    }

    return true;
}

bool ToolPicker::moveObjectToPoolPosition()
{
    if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver")
    {
        return goToPose( 0.20, -0.85, -0.30, POOL_ORI_R);
    }
    else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
    {
        return goToPose( 0.00, -0.85, -0.25, POOL_ORI_R);
    }
    else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box")
    {
        return goToPose(-0.15, -0.85, -0.25, POOL_ORI_R);
    }

    return false;
}

bool ToolPicker::determineContactCondition()
{
    if (hasCollidedIR("strict") || hasCollidedCD())
    {
        if (hasCollidedCD())
        {
            moveArm("up", 0.002);
        }
        ROS_INFO("Collision!");
        return true;
    }
    else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) != "screwdriver")
    {
        if (getAction() == ACTION_CLEANUP)
        {
            if (getPos().z < -0.17)
            {
                ROS_INFO("Object reached!");
                return true;
            }
        }
        else if (getAction() == ACTION_GET      ||
                 getAction() == ACTION_GET_PASS)
        {
            if (getPos().z < -0.28)
            {
                ROS_INFO("Object reached!");
                return true;
            }
        }
    }

    return false;
}

bool ToolPicker::computeOffsets(double &_x_offs, double &_y_offs)
{
    if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver")
        {
            _x_offs = +0.010;
            // _y_offs = +0.017;
        }
        else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box"  ||
                 getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
        {
            _x_offs = +0.06;
        }
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver")
        {
            // _x_offs = -0.020;
            _y_offs = -0.010;
        }
        else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box"  ||
                 getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
        {
            _x_offs = +0.020;
            _y_offs = -0.058;
        }
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET, ACTION_GET_PASS or ACTION_CLEANUP!");
        return false;
    }

    return true;
}

ToolPicker::~ToolPicker()
{

}

#include "artag_hold_ctrl.h"

using namespace              std;
using namespace baxter_core_msgs;

#define VERTICAL_ORI_R2         0.1, 1.0, 0.0, 0.0

ARTagHoldCtrl::ARTagHoldCtrl(string _name, string _limb, bool _use_robot) :
                             ARTagCtrl(_name, _limb, _use_robot)
{
    setHomeConfiguration();

    setState(START);

    insertAction(string(ACTION_HOLD) + "_leg", static_cast<f_action>(&ARTagHoldCtrl::holdObject));
    insertAction(string(ACTION_HOLD) + "_top", static_cast<f_action>(&ARTagHoldCtrl::holdObject));

    printActionDB();

    if (not _use_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ARTagHoldCtrl::pickARTag()
{
    ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());

    if (!isIRok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        setSubState(NO_IR_SENSOR);
        return false;
    }

    if (!waitForData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    double offs_x = 0.0;
    double offs_y = 0.0;

    if (!computeOffsets(offs_x, offs_y))    return false;

    // Let's compute a first estimation of the joint position
    // (we reduce the z by 10 cm to start picking up from a
    // closer position)
    double x = getObjectPos().x + offs_x;
    double y = getObjectPos().y + offs_y;
    double z =       getPos().z -   0.15;

    geometry_msgs::Quaternion q;
    if (!computeOrientation(q))             return false;

    ROS_INFO("Going to: %g %g %g", x, y, z);

    if (!goToPose(x, y, z, q.x, q.y, q.z, q.w, "loose"))
    {
        return false;
    }

    if (!waitForData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int cnt_ik_fail = 0;

    ros::Rate r(THREAD_FREQ);

    while(RobotInterface::ok())
    {
        double new_elap_time = (ros::Time::now() - start_time).toSec();

        x = getObjectPos().x + offs_x;
        y = getObjectPos().y + offs_y;
        z = z_start - ARM_SPEED * new_elap_time / 1.3;

        ROS_DEBUG("Time %g Going to: %g %g %g Position: %g %g %g", new_elap_time, x, y, z,
                                                       getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,q.x, q.y, q.z, q.w))
        {
            cnt_ik_fail = 0;
            // if (new_elap_time - elap_time > 0.02)
            // {
            //     ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            // }
            // elap_time = new_elap_time;

            if (determineContactCondition())
            {
                return true;
            }

            r.sleep();
        }
        else
        {
            ++cnt_ik_fail;
        }

        if (cnt_ik_fail == 10)  return false;
    }

    return false;
}

bool ARTagHoldCtrl::determineContactCondition()
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
    else if (false)
    {
        if (getAction() == ACTION_GET      ||
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

bool ARTagHoldCtrl::computeOffsets(double &_x_offs, double &_y_offs)
{
    if (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        if      (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) ==          "foot")
        {

        }
        else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "bracket_front")
        {

        }
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET or ACTION_GET_PASS!");
        return false;
    }

    return true;
}

bool ARTagHoldCtrl::computeOrientation(geometry_msgs::Quaternion &_q)
{
    if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        quaternionFromDoubles(_q, POOL_ORI_R);
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET or ACTION_GET_PASS!");
        return false;
    }

    return true;
}

ARTagHoldCtrl::~ARTagHoldCtrl()
{

}

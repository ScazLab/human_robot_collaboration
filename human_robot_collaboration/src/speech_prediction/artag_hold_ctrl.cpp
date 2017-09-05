#include "artag_hold_ctrl.h"

using namespace              std;
using namespace baxter_core_msgs;

#define VERTICAL_ORI_R2         0.1, 1.0, 0.0, 0.0

ARTagHoldCtrl::ARTagHoldCtrl(string _name, string _limb, bool _use_robot) :
                             ARTagCtrl(_name, _limb, _use_robot), elap_time(0)
{
    setHomeConfiguration();

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ARTagHoldCtrl::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ARTagHoldCtrl::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ARTagHoldCtrl::getPassObject));

    removeAction(ACTION_HOLD);

    insertAction(string(ACTION_HOLD) + "_leg", static_cast<f_action>(&ARTagHoldCtrl::holdObject));
    insertAction(string(ACTION_HOLD) + "_top", static_cast<f_action>(&ARTagHoldCtrl::holdObject));

    printActionDB();

    if (not _use_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ARTagHoldCtrl::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();

    if (getObjectIDs().size() >  1)
    {
        setSubState(CHECK_OBJ_IDS);
        int id = chooseObjectID(getObjectIDs());
        if (id == -1)       return false;
        setObjectID(id);
        ROS_INFO("[%s] Chosen object with ID %i", getLimb().c_str(),
                                                     getObjectID());
    }

    if (!pickUpObject())            return false;
    if (!close())                   return false;
    if (!moveArm("up", 0.3))        return false;
    // if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool ARTagHoldCtrl::passObject()
{
    if (getPrevAction() != ACTION_GET)  return false;

    if (!goToPose(0.85, -0.26, 0.27,
                  HORIZONTAL_ORI_R))    return false;
    if (!waitForUserCuffUpperFb())      return false;
    if (!open())                        return false;
    if (!homePoseStrict())              return false;

    return true;
}

bool ARTagHoldCtrl::getPassObject()
{
    if (!getObject())      return false;
    setPrevAction(ACTION_GET);
    if (!passObject())     return false;

    return true;
}

bool ARTagHoldCtrl::pickUpObject()
{
    ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());

    if (!isIRok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        setSubState(NO_IR_SENSOR);
        return false;
    }

    if (!waitForARucoData())
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
    double x = getMarkerPos().x + offs_x;
    double y = getMarkerPos().y + offs_y;
    double z =       getPos().z -   0.15;

    geometry_msgs::Quaternion q;
    if (!computeOrientation(q))             return false;

    ROS_INFO("Going to: %g %g %g", x, y, z);

    if (!goToPose(x, y, z, q.x, q.y, q.z, q.w, "loose"))
    {
        return false;
    }

    if (!waitForARucoData())
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

        x = getMarkerPos().x + offs_x;
        y = getMarkerPos().y + offs_y;
        z = z_start - ARM_SPEED * new_elap_time / 1.3;

        ROS_DEBUG("Time %g Going to: %g %g %g Position: %g %g %g", new_elap_time, x, y, z,
                                                       getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,q.x, q.y, q.z, q.w))
        {
            cnt_ik_fail = 0;
            if (new_elap_time - elap_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            }
            elap_time = new_elap_time;

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

bool ARTagHoldCtrl::holdObject()
{
    if (!startHold())            return false;
    if (!endHold())              return false;

    return true;
}

bool ARTagHoldCtrl::startHold()
{
    double time=getObjectIDs().size()>=2?getObjectIDs()[0]:30.0;

    if (!goHoldPose(0.30))              return false;
    ros::Duration(1.0).sleep();
    if (!waitForUserCuffUpperFb(time))  return false;
    if (!close())                       return false;
    ros::Duration(1.0).sleep();

    return true;
}

bool ARTagHoldCtrl::endHold()
{
    double time=getObjectIDs().size()>=2?getObjectIDs()[1]:180.0;

    if (!waitForUserCuffUpperFb(time))  return false;
    if (!open())                        return false;
    ros::Duration(1.0).sleep();
    if (!homePoseStrict())              return false;
    return true;
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

int ARTagHoldCtrl::chooseObjectID(vector<int> _objs)
{
    if (getSubState() != CHECK_OBJ_IDS)
    {
        return ArmCtrl::chooseObjectID(_objs);
    }

    ROS_DEBUG("[%s] Choosing object IDs", getLimb().c_str());
    int res = -1;

    if (!waitForARucoOK())
    {
        setSubState(NO_OBJ);
        return res;
    }

    if (!waitForARucoMarkersFound())
    {
        setSubState(NO_OBJ);
        return res;
    }

    std::vector<int> av_markers = getAvailableMarkers(_objs);

    if (av_markers.size() == 0)     return res;

    srand(time(0)); //use current time as seed
    res = av_markers[rand() % av_markers.size()];

    return res;
}

bool ARTagHoldCtrl::goHoldPose(double height)
{
    ROS_INFO("[%s] Going to %s position..", getLimb().c_str(), getAction().c_str());

    if (getAction() == string(ACTION_HOLD) + "_top")
    {
        return goToPose(0.72, -0.31, 0.032, 0.54, 0.75, 0.29,0.22);
    }

    return goToPose(0.80, -0.4, height, HORIZONTAL_ORI_R);
}

ARTagHoldCtrl::~ARTagHoldCtrl()
{

}

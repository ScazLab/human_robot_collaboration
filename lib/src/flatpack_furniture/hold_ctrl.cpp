#include "flatpack_furniture/hold_ctrl.h"

using namespace std;
using namespace baxter_core_msgs;

HoldCtrl::HoldCtrl(std::string _name, std::string _limb, bool _no_robot) :
                   ArmCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();
    setState(START);

    insertAction(ACTION_START_HOLD, static_cast<f_action>(&HoldCtrl::startHold));
    insertAction(ACTION_END_HOLD,   static_cast<f_action>(&HoldCtrl::endHold));
    insertAction(ACTION_HOLD,       static_cast<f_action>(&HoldCtrl::holdObject));
    insertAction(ACTION_HAND_OVER,  static_cast<f_action>(&HoldCtrl::handOver));

    // Not implemented actions throw a ROS_ERROR and return always false:
    insertAction("recover_"+string(ACTION_START_HOLD), &HoldCtrl::notImplemented);
    insertAction("recover_"+string(ACTION_END_HOLD),   &HoldCtrl::notImplemented);
    insertAction("recover_"+string(ACTION_HOLD),       &HoldCtrl::notImplemented);
    insertAction("recover_"+string(ACTION_HAND_OVER),  &HoldCtrl::notImplemented);

    printDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool HoldCtrl::doAction(int s, std::string a)
{
    if (a == ACTION_HOLD       || a == "recover_"+string(ACTION_HOLD)       ||
        a == ACTION_START_HOLD || a == "recover_"+string(ACTION_START_HOLD) ||
        a == ACTION_END_HOLD   || a == "recover_"+string(ACTION_END_HOLD)   ||
        a == ACTION_HAND_OVER  || a == "recover_"+string(ACTION_HAND_OVER)  ||
        a == "recover_"+string(ACTION_RELEASE))
    {
        if (callAction(a))  return true;
        else                recoverFromError();
    }
    else
    {
        ROS_ERROR("[%s] Invalid Action %s in state %i", getLimb().c_str(), a.c_str(), s);
    }

    return false;
}

bool HoldCtrl::handOver()
{
    setSubState(HAND_OVER_START);
    if (!prepare4HandOver())              return false;
    setSubState(HAND_OVER_READY);
    if (!waitForOtherArm(120.0, true))    return false;
    if (!gripObject())                    return false;
    ros::Duration(1.2).sleep();
    if (!goHoldPose(0.24))                return false;
    // ros::Duration(1.0).sleep();
    // if (!waitForForceInteraction(180.0))  return false;
    // if (!releaseObject())                 return false;
    // ros::Duration(1.0).sleep();
    // if (!homePoseStrict())         return false;
    setSubState("");

    return true;
}


bool HoldCtrl::startHold()
{
    double time=getMarkerID()>0?getMarkerID():30.0;

    if (!goHoldPose(0.30))              return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction(time)) return false;
    if (!gripObject())                  return false;

    return true;
}


bool HoldCtrl::endHold()
{
    double time=getMarkerID()>0?getMarkerID():180.0;

    if (!waitForForceInteraction(time)) return false;
    if (!releaseObject())               return false;
    ros::Duration(1.0).sleep();
    if (!homePoseStrict())       return false;
    return true;
}

bool HoldCtrl::holdObject()
{
    if (!startHold())            return false;
    ros::Duration(1.0).sleep();
    if (!endHold())              return false;

    return true;
}

bool HoldCtrl::waitForOtherArm(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    ros::Rate r(100);
    while(RobotInterface::ok())
    {
        if (disable_coll_av)      suppressCollisionAv();

        if (getSubState() == HAND_OVER_DONE)   return true;

        r.sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("[%s] No feedback from other arm has been received in %gs!",
                                                    getLimb().c_str(), _wait_time);
            return false;
        }
    }
    return false;
}

void HoldCtrl::setHomeConfiguration()
{
    setHomeConf( 0.0717, -1.0009, 1.1083, 1.5520,
                         -0.5235, 1.3468, 0.4464);
}

bool HoldCtrl::serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                                  baxter_collaboration::AskFeedback::Response &res)
{
    res.success = false;
    if (req.ask == HAND_OVER_READY)
    {
        res.success = false;
        if (getSubState() == HAND_OVER_START) res.reply = HAND_OVER_WAIT;
        if (getSubState() == HAND_OVER_READY)
        {
            setSubState(HAND_OVER_DONE);
            res.reply = getSubState();
        }
    }
    return true;
}

bool HoldCtrl::prepare4HandOver()
{
    ROS_INFO("[%s] Preparing for handover..", getLimb().c_str());
    return goToPose(0.61, 0.15, Z_LOW+0.02, HANDOVER_ORI_R);
}

bool HoldCtrl::goHoldPose(double height)
{
    ROS_INFO("[%s] Going to hold position..", getLimb().c_str());
    return goToPose(0.80, -0.4, height, HORIZONTAL_ORI_R);
}

HoldCtrl::~HoldCtrl()
{

}

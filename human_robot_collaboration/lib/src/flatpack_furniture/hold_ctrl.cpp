#include "flatpack_furniture/hold_ctrl.h"

using namespace              std;
using namespace baxter_core_msgs;

HoldCtrl::HoldCtrl(string _name, string _limb, bool _use_robot) :
                   ArmCtrl(_name,_limb, _use_robot)
{
    setHomeConfiguration();
    setState(START);

    insertAction(ACTION_START_HOLD, static_cast<f_action>(&HoldCtrl::startHold));
    insertAction(ACTION_END_HOLD,   static_cast<f_action>(&HoldCtrl::endHold));
    insertAction(ACTION_HOLD,       static_cast<f_action>(&HoldCtrl::holdObject));

    if (not _use_robot) return;
}

bool HoldCtrl::handOver()
{
    setSubState(HAND_OVER_START);
    if (!prepare4HandOver())              return false;
    setSubState(HAND_OVER_READY);
    if (!waitForOtherArm(120.0, true))    return false;
    if (!close())                         return false;
    ros::Duration(1.2).sleep();
    if (!goHandOverPose())                return false;
    // ros::Duration(1.0).sleep();
    // if (!waitForForceInteraction(180.0))  return false;
    // if (!open())                          return false;
    // ros::Duration(1.0).sleep();
    // if (!homePoseStrict())         return false;
    setSubState("");

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
    ArmCtrl::setHomeConfiguration("table");
}

bool HoldCtrl::serviceOtherLimbCb(human_robot_collaboration_msgs::AskFeedback::Request  &req,
                                  human_robot_collaboration_msgs::AskFeedback::Response &res)
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

bool HoldCtrl::goHandOverPose()
{
    ROS_INFO("[%s] Going to handover position..", getLimb().c_str());
    return goToPose(0.80, -0.4, 0.24, HORIZONTAL_ORI_R);
}

HoldCtrl::~HoldCtrl()
{

}

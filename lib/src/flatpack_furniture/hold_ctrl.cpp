#include "flatpack_furniture/hold_ctrl.h"

using namespace std;
using namespace baxter_core_msgs;

HoldCtrl::HoldCtrl(std::string _name, std::string _limb, bool _no_robot) :
                   ArmCtrl(_name,_limb, _no_robot), cuff_button_pressed(false)
{
    setHomeConfiguration();
    setState(START);

    insertAction(ACTION_START_HOLD, static_cast<f_action>(&HoldCtrl::startHold));
    insertAction(ACTION_END_HOLD,   static_cast<f_action>(&HoldCtrl::endHold));
    insertAction(ACTION_HOLD,       static_cast<f_action>(&HoldCtrl::holdObject));

    XmlRpc::XmlRpcValue objects_db;
    if(!_n.getParam("objects_"+getLimb(), objects_db))
    {
        ROS_INFO("No objects' database found in the parameter server. "
                 "Looked up param is %s", ("objects_"+getLimb()).c_str());
    }
    else
    {
        insertObjects(objects_db);
        printObjectDB();
    }

    if (_no_robot) return;
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
    double time=getObjectIDs().size()>=2?getObjectIDs()[0]:30.0;

    if (!goHoldPose(0.30))              return false;
    ros::Duration(1.0).sleep();
    if (!waitForUserFb(time))           return false;
    if (!gripObject())                  return false;
    ros::Duration(1.0).sleep();

    return true;
}

bool HoldCtrl::endHold()
{
    double time=getObjectIDs().size()>=2?getObjectIDs()[1]:180.0;

    if (!waitForUserFb(time))           return false;
    if (!releaseObject())               return false;
    ros::Duration(1.0).sleep();
    if (!homePoseStrict())              return false;
    return true;
}

bool HoldCtrl::waitForUserFb(double _wait_time)
{
    ROS_INFO("[%s] Waiting user feedback for %g [s]",
                      getLimb().c_str(), _wait_time);

    cuff_button_pressed = false;

    ros::Time _init = ros::Time::now();

    ros::Rate(THREAD_FREQ);
    while(RobotInterface::ok())
    {
        if (cuff_button_pressed == true)        return true;

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No user feedback has been detected in %g [s]!",_wait_time);
            return false;
        }
    }

    return false;
}

void HoldCtrl::cuffUpperCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        cuff_button_pressed = true;
    }

    return;
}

bool HoldCtrl::holdObject()
{
    if (!startHold())            return false;
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

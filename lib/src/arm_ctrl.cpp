#include "robot_interface/arm_ctrl.h"
#include <pthread.h>

using namespace std;
using namespace geometry_msgs;

ArmCtrl::ArmCtrl(string _name, string _limb, bool no_robot) : RobotInterface(_limb, no_robot), Gripper(_limb),
                                                              name(_name), marker_id(-1), action(""), sub_state("")
{
    _cuff_sub      = _n.subscribe("/robot/digital_io/" + _limb + "_lower_button/state",
                                    SUBSCRIBER_BUFFER, &ArmCtrl::cuffOKCb, this);

    std::string topic = "/"+getName()+"/state_"+_limb;
    state_pub = _n.advertise<baxter_collaboration::ArmState>(topic,1);
    ROS_INFO("[%s] Created state publisher with name : %s", getLimb().c_str(), topic.c_str());

    std::string other_limb = getLimb() == "right" ? "left" : "right";

    topic = "/"+getName()+"/service_"+_limb;
    service = _n.advertiseService(topic, &ArmCtrl::serviceCb, this);
    ROS_INFO("[%s] Created service server with name : %s", getLimb().c_str(), topic.c_str());

    topic = "/"+getName()+"/service_"+_limb+"_to_"+other_limb;
    service_other_limb = _n.advertiseService(topic, &ArmCtrl::serviceOtherLimbCb,this);
    ROS_INFO("[%s] Created service server with name : %s", getLimb().c_str(), topic.c_str());
}

void ArmCtrl::InternalThreadEntry()
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    std::string a =     getAction();
    int         s = int(getState());

    setState(WORKING);

    if (a == ACTION_HOME)
    {
        if (goHome())   setState(DONE);
    }
    else if (a == ACTION_RELEASE)
    {
        if (releaseObject())   setState(DONE);
    }
    else if (s == START || s == ERROR ||
             s == DONE  || s == KILLED )
    {
        if (!doAction(s, a))   setState(ERROR);
    }

    if (getState()==WORKING)
    {
        setState(ERROR);
    }

    if (getState()==ERROR)
    {
        ROS_ERROR("[%s] Action %s not successful! State %s %s", getLimb().c_str(), a.c_str(),
                                          string(getState()).c_str(), getSubState().c_str());
    }

    closeInternalThread();
    return;
}

void ArmCtrl::cuffOKCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        ROS_DEBUG("[%s] Action Killed!",getLimb().c_str());
        setState(KILLED);
    }
}

bool ArmCtrl::serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                                 baxter_collaboration::AskFeedback::Response &res)
{
    res.success = false;
    res.reply   = "not implemented";
    return true;
}

bool ArmCtrl::serviceCb(baxter_collaboration::DoAction::Request  &req,
                        baxter_collaboration::DoAction::Response &res)
{
    string action = req.action;
    int    ID     = req.object;

    ROS_INFO("[%s] Service request received. Action: %s object: %i", getLimb().c_str(),
                                                                   action.c_str(), ID);
    if (is_no_robot())
    {
        ros::Duration(2.0).sleep();
        res.success = true;
        return true;
    }

    res.success = false;

    setAction(action);
    setMarkerID(ID);

    startInternalThread();
    ros::Duration(0.5).sleep();

    ros::Rate r(100);
    while( ros::ok() && ( int(getState()) != START   &&
                          int(getState()) != ERROR   &&
                          int(getState()) != DONE    &&
                          int(getState()) != PICK_UP   ))
    {
        if (ros::isShuttingDown())
        {
            setState(KILLED);
            return true;
        }

        if (getState()==KILLED)
        {
            goHome();
        }

        r.sleep();
    }

    if ( int(getState()) == START   ||
         int(getState()) == DONE    ||
         int(getState()) == PICK_UP   )
    {
        res.success = true;
    }

    ROS_INFO("[%s] Service reply with success: %s\n", getLimb().c_str(),
                                            res.success?"true":"false");
    return true;
}

bool ArmCtrl::moveArm(string dir, double dist, string mode, bool disable_coll_av)
{
    Point start = getPos();
    Point final = getPos();

    Quaternion ori = getOri();

    if      (dir == "backward") final.x -= dist;
    else if (dir == "forward")  final.x += dist;
    else if (dir == "right")    final.y -= dist;
    else if (dir == "left")     final.y += dist;
    else if (dir == "down")     final.z -= dist;
    else if (dir == "up")       final.z += dist;
    else                               return false;

    ros::Time t_start = ros::Time::now();

    bool finish = false;

    ros::Rate r(100);
    while(RobotInterface::ok())
    {
        if (disable_coll_av)    suppressCollisionAv();

        double t_elap = (ros::Time::now() - t_start).toSec();

        double px = start.x;
        double py = start.y;
        double pz = start.z;

        if (!finish)
        {
            if (dir == "backward" | dir == "forward")
            {
                int sgn = dir=="backward"?-1:+1;
                px = px + sgn * PICK_UP_SPEED * t_elap;

                if (dir == "backward")
                {
                    if (px < final.x) finish = true;
                }
                else if (dir == "forward")
                {
                    if (px > final.x) finish = true;
                }
            }
            if (dir == "right" | dir == "left")
            {
                int sgn = dir=="right"?-1:+1;
                py = py + sgn * PICK_UP_SPEED * t_elap;

                if (dir == "right")
                {
                    if (py < final.y) finish = true;
                }
                else if (dir == "left")
                {
                    if (py > final.y) finish = true;
                }
            }
            if (dir == "down" | dir == "up")
            {
                int sgn = dir=="down"?-1:+1;
                pz = pz + sgn * PICK_UP_SPEED * t_elap;

                if (dir == "down")
                {
                    if (pz < final.z) finish = true;
                }
                else if (dir == "up")
                {
                    if (pz > final.z) finish = true;
                }
            }
        }
        else
        {
            px = final.x;
            py = final.y;
            pz = final.z;
        }

        double ox = ori.x;
        double oy = ori.y;
        double oz = ori.z;
        double ow = ori.w;

        vector<double> joint_angles;
        if (!callIKService(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

        if (!goToPoseNoCheck(joint_angles))   return false;

        if(mode == "strict")
        {
            if(withinThres(getPos().x, final.x, 0.001) &&
               withinThres(getPos().y, final.y, 0.001) &&
               withinThres(getPos().z, final.z, 0.001)) return true;
        }
        else if(mode == "loose")
        {
            if(withinThres(getPos().x, final.x, 0.01) &&
               withinThres(getPos().y, final.y, 0.01) &&
               withinThres(getPos().z, final.z, 0.01)) return true;
        }

        r.sleep();
    }

    return false;
}

void ArmCtrl::insertAction(const std::string &a, ArmCtrl::f_action f)
{
    action_db.insert( std::make_pair( a, f ));
}

void ArmCtrl::removeAction(const std::string &a)
{
    action_db.erase(a);
}

bool ArmCtrl::callAction(const std::string &a)
{
    f_action act = action_db[a];
    return (this->*act)();
}

bool ArmCtrl::hoverAboveTable(double height, string mode, bool disable_coll_av)
{
    if (getLimb() == "right")
    {
        return RobotInterface::goToPose(HOME_POS_R, height, VERTICAL_ORI_R,
                                                     mode, disable_coll_av);
    }
    else if (getLimb() == "left")
    {
        return RobotInterface::goToPose(HOME_POS_L, height, VERTICAL_ORI_L,
                                                     mode, disable_coll_av);
    }
    else return false;
}

bool ArmCtrl::goHome()
{
    bool res = hoverAboveTableStrict();
    releaseObject();
    return res;
}

void ArmCtrl::recoverFromError()
{
    goHome();
}

void ArmCtrl::setState(int _state)
{
    if (_state == KILLED && getState() != WORKING)
    {
        ROS_WARN("[%s] Attempted to kill a non-working controller", getLimb().c_str());
        return;
    }

    RobotInterface::setState(_state);

    // if (_state == DONE)
    // {
    //     setAction("");
    //     setMarkerID(-1);
    // }
    if (_state == DONE)
    {
        setSubState(getAction());
    }
    publishState();
}

void ArmCtrl::setAction(string _action)
{
    action = _action;
    publishState();
}

void ArmCtrl::publishState()
{
    baxter_collaboration::ArmState msg;

    msg.state  = string(getState());
    msg.action = getAction();
    msg.object = getObjName();

    state_pub.publish(msg);
}

string ArmCtrl::getObjName()
{
    if      (marker_id == 17) return "left leg";
    else if (marker_id == 21) return "top";
    else if (marker_id == 24) return "central frame";
    else if (marker_id == 26) return "right leg";
    else return "";
}

ArmCtrl::~ArmCtrl()
{
    killInternalThread();
}

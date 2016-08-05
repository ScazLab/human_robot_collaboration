#include "robot_interface/arm_ctrl.h"

using namespace std;

ArmCtrl::ArmCtrl(string _name, string _limb) : ROSThread(_limb), Gripper(_limb),
                                               marker_id(-1), name(_name), action("")
{
    std::string other_limb = getLimb() == "right" ? "left" : "right";

    std::string service_name = "/"+name+"/service_"+_limb;
    service = _n.advertiseService(service_name, &ArmCtrl::serviceCb, this);
    ROS_INFO("[%s] Created service server with name : %s", getLimb().c_str(),
                                                        service_name.c_str());

    service_name = "/"+name+"/service_"+_limb+"_to_"+other_limb;
    service_other_limb = _n.advertiseService(service_name, &ArmCtrl::serviceOtherLimbCb,this);
    ROS_INFO("[%s] Created service server with name : %s", getLimb().c_str(),
                                                        service_name.c_str());
}

void ArmCtrl::InternalThreadEntry()
{
    std::string a =     getAction();
    int         s = int(getState());

    setState(WORKING);

    if (a == ACTION_HOME)
    {
        if (goHome())   setState(START);
    }
    else if (a == ACTION_RELEASE)
    {
        if (releaseObject())   setState(START);
    }
    else
    {
        if (!doAction(s, a))   setState(ERROR);
    }

    pthread_exit(NULL);
    return;
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

    ROS_INFO("[%s] Service request received. Action: %s object: %i",
                                                  getLimb().c_str(),
                                                action.c_str(), ID);

    res.success = false;

    setAction(action);
    setMarkerID(ID);

    startInternalThread();
    ros::Duration(0.5).sleep();

    while( int(getState()) != START   &&
           int(getState()) != ERROR   &&
           int(getState()) != DONE    &&
           int(getState()) != PICK_UP   )
    {
        ros::spinOnce();
        ros::Rate(100).sleep();
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
    return true;
}

bool ArmCtrl::hoverAboveTable(double height, string mode, bool disable_coll_av)
{
    if (getLimb() == "right")
    {
        return ROSThread::goToPose(HOME_POS_R, height, VERTICAL_ORI_R,
                                                mode, disable_coll_av);
    }
    else if (getLimb() == "left")
    {
        return ROSThread::goToPose(HOME_POS_L, height, VERTICAL_ORI_L,
                                                mode, disable_coll_av);
    }
}

bool ArmCtrl::goHome()
{
    return hoverAboveTable(Z_LOW);
}

void ArmCtrl::recoverFromError()
{
    releaseObject();
    goHome();
}

ArmCtrl::~ArmCtrl()
{

}

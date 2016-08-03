#include "robot_interface/arm_ctrl.h"

using namespace std;

ArmCtrl::ArmCtrl(string _name, string _limb) : ROSThread(_limb), Gripper(_limb),
                                               name(_name), action("")
{
    service = _n.advertiseService("/"+name+"/action_service_"+_limb,
                                   &ArmCtrl::serviceCb, this);
}

bool ArmCtrl::serviceCb(baxter_collaboration::DoAction::Request  &req,
                        baxter_collaboration::DoAction::Response &res)
{
    string action = req.action;
    int    ID     = req.object;

    ROS_INFO("Service request received. Action: %s object: %i", action.c_str(), ID);

    res.success = false;

    setState(WORKING);
    setAction(action);
    startInternalThread();
    ros::Duration(1.0).sleep();

    while( int(getState()) != START   &&
           int(getState()) != ERROR   &&
           int(getState()) != DONE    &&
           int(getState()) != PICK_UP   )
    {
        ros::spinOnce();
    }

    if ( int(getState()) == START   ||
         int(getState()) == DONE    ||
         int(getState()) == PICK_UP   )
    {
        res.success = true;
    }

    ROS_INFO("Service reply with success: %s\n", res.success?"true":"false");
    return true;
}

void ArmCtrl::recoverFromError()
{
    releaseObject();
    goHome();
    setState(ERROR);
}

ArmCtrl::~ArmCtrl()
{

}

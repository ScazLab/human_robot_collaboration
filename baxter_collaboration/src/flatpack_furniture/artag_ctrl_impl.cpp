#include "artag_ctrl_impl.h"

ARTagCtrlImpl::ARTagCtrlImpl(std::string _name, std::string _limb, bool _use_robot) :
                             ARTagCtrl(_name,_limb, _use_robot)
{
    setHomeConfiguration();

    insertAction(ACTION_HAND_OVER, static_cast<f_action>(&ARTagCtrlImpl::handOver));

    insertAction("recover_"+std::string(ACTION_RELEASE),
                 static_cast<f_action>(&ARTagCtrlImpl::recoverRelease));

    insertAction("recover_"+std::string(ACTION_GET),
                  static_cast<f_action>(&ARTagCtrlImpl::recoverGet));

    // Not implemented actions throw a ROS_ERROR and return always false:
    insertAction("recover_"+std::string(ACTION_PASS),      &ARTagCtrlImpl::notImplemented);
    insertAction("recover_"+std::string(ACTION_HAND_OVER), &ARTagCtrlImpl::notImplemented);

    printActionDB();

    if (_use_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);

    // moveArm("up",0.2,"strict");
    // moveArm("down",0.2,"strict");
    // moveArm("right",0.2,"strict");
    // moveArm("left",0.2,"strict");
    // moveArm("forward",0.1,"strict");
    // moveArm("backward",0.2,"strict");
    // moveArm("forward",0.1,"strict");
}

ARTagCtrlImpl::~ARTagCtrlImpl()
{

}

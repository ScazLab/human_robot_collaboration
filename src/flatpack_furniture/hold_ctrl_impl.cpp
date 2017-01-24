#include "hold_ctrl_impl.h"

HoldCtrlImpl::HoldCtrlImpl(std::string _name, std::string _limb, bool _no_robot) :
                           HoldCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();

    insertAction(ACTION_HAND_OVER,  static_cast<f_action>(&HoldCtrlImpl::handOver));

    // Not implemented actions throw a ROS_ERROR and return always false:
    insertAction("recover_"+std::string(ACTION_START_HOLD), &HoldCtrlImpl::notImplemented);
    insertAction("recover_"+std::string(ACTION_END_HOLD),   &HoldCtrlImpl::notImplemented);
    insertAction("recover_"+std::string(ACTION_HOLD),       &HoldCtrlImpl::notImplemented);
    insertAction("recover_"+std::string(ACTION_HAND_OVER),  &HoldCtrlImpl::notImplemented);

    printActionDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

HoldCtrlImpl::~HoldCtrlImpl()
{

}

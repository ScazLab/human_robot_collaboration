#include "robot_interface/gripper.h"

#include <iostream>

using namespace baxter_core_msgs;
using namespace std;

Gripper::Gripper(std::string limb, bool no_robot) :
                 _limb(limb), _no_robot(no_robot), _first_run(true)
{
    if (no_robot) return;

    _pub_command = _nh.advertise<EndEffectorCommand>(
                   "/robot/end_effector/" + _limb + "_gripper/command", 1);

    _sub_state = _nh.subscribe("/robot/end_effector/" + _limb + "_gripper/state",
                                SUBSCRIBER_BUFFER, &Gripper::gripperCb, this);

    //Initially all the interesting properties of the state are unknown
    EndEffectorState initial_gripper_state;
    initial_gripper_state.calibrated=   \
    initial_gripper_state.enabled=      \
    initial_gripper_state.error=        \
    initial_gripper_state.gripping=     \
    initial_gripper_state.missed=       \
    initial_gripper_state.ready=        \
    initial_gripper_state.moving=EndEffectorState::STATE_UNKNOWN;

    _state = initial_gripper_state;

    pthread_mutex_init(&_mutex, NULL);
}

bool Gripper::gripObject()
{
    suck();

    // int cnt = 0;

    // while (!_gripper->is_sucking())
    // {
    //     ROS_WARN("Requested a suck to the gripper, but the gripper is not sucking.");
    //     ++cnt;

    //     if (cnt == 10)  return false;

    //     pause();
    //     ros::spinOnce();
    // }

    return true;
}

bool Gripper::releaseObject()
{
    if (is_sucking())
    {
        blow();
        return true;
    }

    ROS_WARN("[%s_gripper] Requested a release of the gripper, but the gripper is not sucking.", getGripperLimb().c_str());
    return false;
}

void Gripper::gripperCb(const EndEffectorState &msg)
{
    pthread_mutex_lock(&_mutex);
    _state = msg;
    pthread_mutex_unlock(&_mutex);

    if (_first_run)
    {
        if (!is_calibrated())
        {
            ROS_INFO("[%s_gripper] Calibrating the gripper..", getGripperLimb().c_str());
            calibrate();
        }
        _first_run=false;
    }
}

void Gripper::calibrate()
{
    EndEffectorCommand sucking_command;
    sucking_command.id=get_id();
    sucking_command.command=EndEffectorCommand::CMD_CALIBRATE;
    _pub_command.publish(sucking_command);
}

void Gripper::suck()
{
    EndEffectorCommand sucking_command;
    sucking_command.id=get_id();
    sucking_command.command=EndEffectorCommand::CMD_GRIP;
    if (_limb == "left")
    {
        sucking_command.args="{\"grip_attempt_seconds\": 5.0}";
    }
    _pub_command.publish(sucking_command);
}

void Gripper::blow()
{
    EndEffectorCommand release_command;
    release_command.id=get_id();
    release_command.command=EndEffectorCommand::CMD_RELEASE;
    _pub_command.publish(release_command);
}

int Gripper::get_id()
{
    return _state.id;
}

bool Gripper::is_enabled()
{
    return _state.enabled==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_calibrated()
{
    return _state.calibrated==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_ready_to_grip()
{
    return _state.ready==EndEffectorState::STATE_TRUE;
}

bool Gripper::has_error()
{
    return _state.error==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_sucking()
{
    // ROS_INFO("force is: %g\n",_state.force);
    return _state.position<80;
}

bool Gripper::is_gripping()
{
    return true;
    return _state.gripping==EndEffectorState::STATE_TRUE;
}

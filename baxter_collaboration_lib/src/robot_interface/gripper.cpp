#include "robot_interface/gripper.h"

#include <iostream>

using namespace baxter_core_msgs;
using namespace std;

Gripper::Gripper(std::string _limb, bool _use_robot) : limb(_limb), use_robot(_use_robot),
                                           first_run(true), state(new EndEffectorState())
{
    if (not use_robot) return;

    pub = nh.advertise<EndEffectorCommand>(
                   "/robot/end_effector/" + _limb + "_gripper/command", 1);

    sub = nh.subscribe("/robot/end_effector/" + _limb + "_gripper/state",
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

    state.reset(new EndEffectorState(initial_gripper_state));
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

    ROS_WARN("[%s_gripper] Requested a release of the gripper, "
             "but the gripper is not sucking.", getGripperLimb().c_str());
    return false;
}

void Gripper::gripperCb(const EndEffectorState &msg)
{
    state.reset(new EndEffectorState(msg));

    if (first_run)
    {
        if (!is_calibrated())
        {
            ROS_INFO("[%s_gripper] Calibrating the gripper..",
                                    getGripperLimb().c_str());
            calibrate();
        }

        first_run=false;
    }
}

void Gripper::calibrate()
{
    EndEffectorCommand cmd;
    cmd.id=get_id();
    cmd.command=EndEffectorCommand::CMD_CALIBRATE;
    pub.publish(cmd);
}

void Gripper::suck()
{
    EndEffectorCommand cmd;
    cmd.id=get_id();
    cmd.command=EndEffectorCommand::CMD_GRIP;
    if (limb == "left")
    {
        cmd.args="{\"grip_attempt_seconds\": 5.0}";
    }
    pub.publish(cmd);
}

void Gripper::blow()
{
    EndEffectorCommand release_command;
    release_command.id=get_id();
    release_command.command=EndEffectorCommand::CMD_RELEASE;
    pub.publish(release_command);
}

int Gripper::get_id()
{
    return state->id;
}

bool Gripper::is_enabled()
{
    return state->enabled==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_calibrated()
{
    return state->calibrated==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_ready_to_grip()
{
    return state->ready==EndEffectorState::STATE_TRUE;
}

bool Gripper::has_error()
{
    return state->error==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_sucking()
{
    // ROS_INFO("force is: %g\n",state->force);
    return state->position<80;
}

bool Gripper::is_gripping()
{
    return true;
    // return state->gripping==EndEffectorState::STATE_TRUE;
}

Gripper::~Gripper()
{

}

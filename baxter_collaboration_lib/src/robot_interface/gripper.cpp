#include "robot_interface/gripper.h"

#include <iostream>

using namespace baxter_core_msgs;
using namespace std;

Gripper::Gripper(std::string _limb, bool _use_robot) :
                 limb(_limb), use_robot(_use_robot), first_run(true)
{
    if (not use_robot) return;

    pub = rnh.advertise<EndEffectorCommand>(
                   "/robot/end_effector/" + _limb + "_gripper/command", 1);

    sub = rnh.subscribe("/robot/end_effector/" + _limb + "_gripper/state",
                           SUBSCRIBER_BUFFER, &Gripper::gripperCb, this);

    //Initially all the interesting properties of the state are unknown
    EndEffectorState init_state;
    init_state.calibrated = EndEffectorState::STATE_UNKNOWN;
    init_state.enabled    = EndEffectorState::STATE_UNKNOWN;
    init_state.error      = EndEffectorState::STATE_UNKNOWN;
    init_state.gripping   = EndEffectorState::STATE_UNKNOWN;
    init_state.missed     = EndEffectorState::STATE_UNKNOWN;
    init_state.ready      = EndEffectorState::STATE_UNKNOWN;
    init_state.moving     = EndEffectorState::STATE_UNKNOWN;

    setGripperState(init_state);

    // create a subscriber to the gripper's properties
    // since the properties are fixed ,the callback is unsubscribed immediately afterwards
    sub_prop = rnh.subscribe("/robot/end_effector/" + _limb + "_gripper/properties",
                                SUBSCRIBER_BUFFER, &Gripper::gripperCbProp, this);
    sub_prop.shutdown();

}

void Gripper::setGripperState(const baxter_core_msgs::EndEffectorState& _state)
{
    std::lock_guard<std::mutex> lock(mutex);
    state = _state;
}

baxter_core_msgs::EndEffectorState Gripper::getGripperState()
{
    std::lock_guard<std::mutex> lock(mutex);
    return state;
}

void Gripper::setGripperProperties(const baxter_core_msgs::EndEffectorProperties& _properties)
{
    std::lock_guard<std::mutex> lock(mutex);
    properties = _properties;
}

baxter_core_msgs::EndEffectorProperties Gripper::getGripperProperties()
{
    std::lock_guard<std::mutex> lock(mutex);
    return properties;
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
    setGripperState(msg);

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

void Gripper::gripperCbProp(const EndEffectorProperties &msg)
{
    setGripperProperties(msg);
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
    return getGripperState().id;
}

bool Gripper::is_enabled()
{
    return getGripperState().enabled==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_calibrated()
{
    return getGripperState().calibrated==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_ready_to_grip()
{
    return getGripperState().ready==EndEffectorState::STATE_TRUE;
}

bool Gripper::has_error()
{
    return getGripperState().error==EndEffectorState::STATE_TRUE;
}

bool Gripper::is_sucking()
{
    // ROS_INFO("force is: %g\n",getGripperState().force);
    return getGripperState().position<80;
}

bool Gripper::is_gripping()
{
    return true;
    // return getGripperState().gripping==EndEffectorState::STATE_TRUE;
}

// sarim's edits:

bool Gripper::reboot()
{
    return true;
}

bool Gripper::cmd_reboot()
{
    return true;
}

std::string Gripper::type()
{
    int ui_code = getGripperProperties().ui_type;
    if(ui_code == 1)
    {
        return "suction";
    }
    else if(ui_code == 2)
    {
        return "electric";
    }
    else
    {
        return "custom";
    }
}




Gripper::~Gripper()
{

}

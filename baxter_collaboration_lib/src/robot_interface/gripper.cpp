#include "robot_interface/gripper.h"

#include <iostream>
#include <string>

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
    sub_prop = rnh.subscribe("/robot/end_effector/" + _limb + "_gripper/properties",
                                SUBSCRIBER_BUFFER, &Gripper::gripperPropCb, this);
}

void Gripper::setGripperState(const baxter_core_msgs::EndEffectorState& _state)
{
    std::lock_guard<std::mutex> lock(mutex_state);
    state = _state;
}

baxter_core_msgs::EndEffectorState Gripper::getGripperState()
{
    std::lock_guard<std::mutex> lock(mutex_state);
    return state;
}

void Gripper::setGripperProperties(const baxter_core_msgs::EndEffectorProperties& _properties)
{
    std::lock_guard<std::mutex> lock(mutex_properties);
    properties = _properties;
}

baxter_core_msgs::EndEffectorProperties Gripper::getGripperProperties()
{
    std::lock_guard<std::mutex> lock(mutex_properties);
    return properties;
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

void Gripper::gripperPropCb(const EndEffectorProperties &msg)
{
    setGripperProperties(msg);
}

void Gripper::calibrate()
{
    EndEffectorCommand cmd;
    cmd.id=get_id();
    cmd.command=EndEffectorCommand::CMD_CALIBRATE;
    pub.publish(cmd);
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

std::string Gripper::type()
{
    int ui_code = (int) getGripperProperties().ui_type;

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

void Gripper::open(bool _block, double _timeout)
{
    if(type() == "electric")
    {
        commandPosition(95.0, _block, _timeout);
    }
    else if (type() == "suction")
    {
        stop(_block, _timeout);
    }
    else
    {
        capabilityWarning("open");
    }
}

void Gripper::close(bool _block, double _timeout)
{
    if(type() == "electric")
    {
        commandPosition(5.0, _block, _timeout);
    }
    else if (type() == "suction")
    {
        commandSuction(_block, _timeout);
    }
    else
    {
        capabilityWarning("close");
    }
}

void Gripper::commandPosition(double _position, bool _block, double _timeout)
{
    if(type() != "electric")
    {
        capabilityWarning("commandPosition");
    }
    if(_position >= 0.0 && _position <= 100.0)
    {
        std::string position_cmd = EndEffectorCommand::CMD_GO;
        std::string position_args =
            "{\"position\": " + std::to_string(_position) + "}";
        command(position_cmd, _block, _timeout, position_args);
    }
    else
    {
        ROS_WARN("Gripper position can only be between 0.0 and 100.0");
    }
}

void Gripper::commandSuction(bool _block, double _timeout)
{
    if(type() != "suction")
    {
        capabilityWarning("commandSuction");
    }
    std::string suction_cmd = EndEffectorCommand::CMD_GO;
    std::string suction_args =
        "{\"grip_attempt_seconds\": " + std::to_string(_timeout) + "}";// default timeout=5.0
    command(suction_cmd, _block, _timeout, suction_args);
}

void Gripper::stop(bool _block, double _timeout)
{
    std::string stop_cmd;

    if(type() == "electric")
    {
        stop_cmd = EndEffectorCommand::CMD_STOP;
    }
    else if(type() == "suction")
    {
        stop_cmd = EndEffectorCommand::CMD_RELEASE;
    }
    else
    {
        capabilityWarning("stop");
    }

    command(stop_cmd, _block, _timeout);
}

void Gripper::command(std::string _cmd, bool _block,
                      double _timeout, std::string _args)
{
    EndEffectorCommand ee_cmd;
    ee_cmd.id = get_id();
    ee_cmd.command = _cmd;
    ee_cmd.args = "";

    if(_args != "")
    {
        ee_cmd.args = _args;
    }

    pub.publish(ee_cmd);

    if(_block == true)
    {
        sleep(_timeout);
    }
}

void Gripper::capabilityWarning(std::string _function)
{
    ROS_WARN("%s gripper of type %s is not capable of %s",
              getGripperLimb().c_str(), type().c_str(), _function.c_str());
}

void Gripper::reboot()
{
    EndEffectorCommand reboot;
    reboot.id=get_id();
    reboot.command=EndEffectorCommand::CMD_REBOOT;
    pub.publish(reboot);
    sleep(1.0);
}

/** Legacy */

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

void Gripper::suck()
{
    if(type() != "suction")
    {
        ROS_INFO("warning: this is not a suction gripper");
    }

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

Gripper::~Gripper()
{

}

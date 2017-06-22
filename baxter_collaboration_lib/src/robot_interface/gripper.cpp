#include "robot_interface/gripper.h"

#include <iostream>
#include <string>

using namespace baxter_core_msgs;
using namespace std;

Gripper::Gripper(std::string _limb, bool _use_robot) :
                 limb(_limb), use_robot(_use_robot), first_run(true), rnh(_limb),
                 spinner(1), cmd_sequence(0), cmd_sender(ros::this_node::getName())
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

    spinner.start();

    // sleep to wait for the publisher to be ready
    ros::Duration(0.5).sleep();

    // set the gripper parameters to their defaults
    setParameters("", true);
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

void Gripper::setParameters(std::string _parameters, bool _defaults)
{
    if(_defaults)
    {
        parameters = validParameters();
    }
    else
    {
        if(_parameters != "")
        {
            // some error checking needed here to prevent invalid parameters being set
            // this would involve converting between strings and dict-type objects
            // and comparing to valid_parameters' keys
            parameters = _parameters;
        }
    }
    // send the parameters to the gripper
    std::string param_cmd = EndEffectorCommand::CMD_CONFIGURE;
    command(param_cmd, false, 0.0, parameters);
}

std::string Gripper::validParameters()
{
    std::string valid;

    if(type() == "electric")
    {
        valid = "{\"velocity\" : 50.0, \"moving_force\" : 40.0, \"holding_force\" : 30.0, \"dead_zone\" : 5.0}";
    }
    else if(type() == "suction")
    {
        valid = "{\"vacuum_sensor_threshold\" : 18.0, \"blow_off_seconds\" : 0.4}";
    }
    else
    {
        valid = "";
    }

    return valid;
}

void Gripper::calibrate()
{
    if(type() != "electric") { capabilityWarning("calibrate"); }
    std::string calibrate_cmd = EndEffectorCommand::CMD_CALIBRATE;
    command(calibrate_cmd, false, 0.0, "");
}

void Gripper::clearCalibration()
{
    if(type() != "electric") { capabilityWarning("clearCalibration"); }
    std::string clear_calibrate_cmd = EndEffectorCommand::CMD_CLEAR_CALIBRATION;
    command(clear_calibrate_cmd, false, 0.0, "");
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
    if(type() != "suction") { capabilityWarning("is_sucking"); }

    // ROS_INFO("force is: %g\n",getGripperState().force);
    return getGripperState().position<80;
}

bool Gripper::is_gripping()
{
    return true;
    // return getGripperState().gripping==EndEffectorState::STATE_TRUE;
}

bool Gripper::hasForce()
{
    return getGripperProperties().controls_force == true;
}

bool Gripper::hasPosition()
{
    return getGripperProperties().controls_position == true;
}

void Gripper::open(bool _block, double _timeout)
{
    if(type() == "electric")
    {
        commandPosition(100.0, _block, _timeout);
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

    // ensures that the gripper is not used without calibration
    if(not is_calibrated())
    {
        ROS_WARN("Cannot issue commandPosition to gripper without calibration");
        return;
    }

    // ensures that the gripper is positioned within physical limits
    if(_position >= 0.0 && _position <= 100.0)
    {
        std::string position_cmd = EndEffectorCommand::CMD_GO;
        std::string position_args = "{\"position\": " +
                                     std::to_string(_position) + "}";

        command(position_cmd, _block, _timeout, position_args);
    }
    else
    {
        ROS_WARN("Gripper position must be between 0.0 and 100.0");
    }
}

void Gripper::commandSuction(bool _block, double _timeout)
{
    if(type() != "suction")
    {
        capabilityWarning("commandSuction");
    }

    std::string suction_cmd = EndEffectorCommand::CMD_GO;
    std::string suction_args = "{\"grip_attempt_seconds\": " +
                                std::to_string(_timeout) + "}";

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
    ee_cmd.sender = cmd_sender;
    ee_cmd.sequence = incCmdSeq();
    ee_cmd.args = "";
    if(_args != "")
    {
        ee_cmd.args = _args;
    }

    pub.publish(ee_cmd);

    if(_block)
    {
        ros::Duration timeout(_timeout);
        wait(timeout);
    }
}

void Gripper::capabilityWarning(std::string _function)
{
    ROS_WARN("%s gripper of type %s is not capable of %s",
              getGripperLimb().c_str(), type().c_str(), _function.c_str());
}

int Gripper::incCmdSeq()
{
    // prevents cmd_sequence from overflowing the integer limit
    cmd_sequence = (cmd_sequence % std::numeric_limits<int>::max()) + 1;
    return cmd_sequence;
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
        return "uninitialized";
    }
}

void Gripper::wait(ros::Duration _timeout)
{
    // waits until the difference between the start and
    // current time catches up to the timeout
    ros::Rate r(100);
    ros::Time start = ros::Time::now();
    while(ros::ok() && ros::Time::now() - start < _timeout)
    {
        ros::spinOnce();
        r.sleep();
    }
}

/** Legacy */

// recommended to use close() instead
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

// recommended to use open() instead
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

// recommended to use commandSuction() instead
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

// recommended to use stop() instead
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

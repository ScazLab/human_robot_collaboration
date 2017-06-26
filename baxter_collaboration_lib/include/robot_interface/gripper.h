#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <mutex>
#include <limits>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorProperties.h>

#include "robot_utils/utils.h"

class Gripper
{
private:
    ros::NodeHandle rnh;    // ROS node handle
    std::string    limb;    // Limb of the gripper: left or right

    bool   use_robot;       // Flag to know if we're going to use the robot or not
    bool   first_run;       // Flag to calibrate the gripper at startup if needed
    bool    prop_set;       // Flag to know if the properties were successfully set

    ros::Subscriber sub_state; // Subscriber to receive the state of the gripper
    ros::Subscriber  sub_prop; // Subscriber to receive the properties of the gripper
    ros::Publisher    pub_cmd; // Publisher for requesting actions to the gripper

    ros::AsyncSpinner spinner; // AsyncSpinner to handle callbacks

    baxter_core_msgs::EndEffectorState           state; // State of the gripper
    baxter_core_msgs::EndEffectorProperties properties; // properties of the gripper
    std::mutex      mutex_state;                        // mutex for controlled state access
    std::mutex mutex_properties;                        // mutex for controlled properties access

    std::string parameters; // parameters for the gripper

    int       cmd_sequence; // counter that tracks the sequence of gripper commands
    std::string cmd_sender; // retains the name of the node sending gripper commands

    /**
     * Callback that handles the gripper state messages.
     */
    void gripperCb(const baxter_core_msgs::EndEffectorState &msg);

    /**
     * Sets the state to the new state, thread-safely
     *
     * @param _state the new state
     */
    void setGripperState(const baxter_core_msgs::EndEffectorState& _state);

    /**
     * Callback that handles the gripper properties messages
     */
    void gripperPropCb(const baxter_core_msgs::EndEffectorProperties &msg);

    /**
     * Sets properties to the new properties, thread-safely
     *
     * @param _properties the new properties
     */
    void setGripperProperties(const baxter_core_msgs::EndEffectorProperties& _properties);

    /**
     * Gets the ID of the gripper
     *
     * @return The ID of the gripper
     */
    int get_id();

    /**
     * Stop the gripper at the current position and apply the holding force
     *
     * @param _block   is the command blocking or non-blocking
     * @param _timeout timeout in seconds for command success
     * @return true/false if success/failure
     */
    bool stop(bool _block=true, double _timeout=5.0);

    /**
     * Command the gripper suction
     *
     * @param _block   is the command blocking or non-blocking
     * @param _timeout timeout in seconds for command success
     * @return true/false if success/failure
     */
    bool commandSuction(bool _block=false, double _timeout=5.0);

    /**
     * Command the gripper position movement
     *
     * @param _position in % 0=close, 100=open
     * @param _block    is the command blocking or non-blocking
     * @param _timeout  timeout in seconds for command success
     * @return true/false if success/failure
     */
    bool commandPosition(double _position, bool _block=false, double _timeout=5.0);

    /**
     * Raw command call to directly control gripper
     *
     * @param _cmd     string of known gripper commands
     * @param _block   is the command blocking or non-blocking
     * @param _timeout timeout in seconds for command evaluation
     * @param _args    parameters and their values in JSON
     * @return true/false if success/failure
     */
    bool command(std::string _cmd, bool _block=false,
                 double _timeout=0.0, std::string _args="");

    /**
     * Set the parameters that will describe the position command execution
     *
     * @param _parameters key-value pairs of parameters
     * @param _defaults   to set or not set the parameters described in validParameters
     */
    void setParameters(std::string _parameters="", bool _defaults=true);

    /**
     * Returns dict of available gripper parameters with default parameters
     *
     * @return valid parameters as a string, but mimicking a Python/JSON dict
     */
    std::string validParameters();

    /**
     * Manage roll over with safe value
     *
     * @return cmd_sequence counter
     */
    int incCmdSeq();

    /**
     * Warns user about functions beyond the capability of a gripper type
     */
    void capabilityWarning(std::string _function);

    /**
     * Waits until the timeout is complete
     *
     * @param _timeout the number of seconds to wait
     */
    bool wait(ros::Duration _timeout);

public:
    /**
     * Constructor of the class
     *
     * @param      _limb either left or right
     * @param _use_robot if to use the robot or not
     **/
    explicit Gripper(std::string _limb, bool _use_robot = true);

    /**
     * Gets the state of the gripper, thread-safely.
     *
     * @return the state of the gripper
     */
    baxter_core_msgs::EndEffectorState getGripperState();

    /**
     * Gets the properties of the gripper, thread-safely
     *
     * @return the properties of the gripper
     */
    baxter_core_msgs::EndEffectorProperties getGripperProperties();

    /**
     * Commands minimum gripper position
     *
     * @param _block   is the command blocking or non-blocking
     * @param _timeout timeout in seconds for open command success
     * @return true/false if success/failure
     */
    bool close(bool _block=false, double _timeout=5.0);

    /**
     * Commands maximum gripper position
     *
     * @param _block   is the command blocking or non-blocking
     * @param _timeout timeout in seconds for open command success
     * @return true/false if success/failure
     */
    bool open(bool _block=false, double _timeout=5.0);

    /**
     * Returns a value indicating if the vacuum gripper is enable, so it can be operated.
     *
     * @return true/false if enabled or not.
     **/
    bool is_enabled();

    /**
     * Indicating if the gripper is calibrated.
     *
     * @return True/false if calibrated or not.
     **/
    bool is_calibrated();

    /**
     * Returns a value indicating if the vacuum gripper is ready for gripping.
     *
     * @return True if it is ready, false otherwise.
     **/
    bool is_ready_to_grip();

    /**
     * Returns a value indicating if the gripper is in error state.
     *
     * @return True if the gripper is in error state, false otherwise.
     **/
    bool has_error();

    /**
     * Returns a value indicating if the vacuum gripper is sucking.
     *
     * @return True if it is sucking, false otherwise.
     **/
    bool is_sucking();

    /**
     * Returns a value indicating if the gripper has something attached.
     *
     * @return True if it is gripping an object, false otherwise.
     **/
    bool is_gripping();

    /**
     * @return bool describing if the gripper is capable of force control
     */
    bool hasForce();

    /**
     * @return bool describing if the gripper is capable of position control
     */
    bool hasPosition();

    /**
     * Returns the limb the gripper belongs to
     *
     * @return the limb, either "left" or "right"
     **/
    std::string getGripperLimb()  { return limb; };

    /**
     * Returns the type of the gripper
     *
     * @return the type, "electric", "suction" or "uninitialized"
     */
    std::string type();

    /**
     * Calibrates the gripper on startup if not already calibrated.
     *
     * @param _block   is the command blocking or non-blocking
     * @param _timeout timeout in seconds for command evaluation
     */
    void calibrate(bool _block=false, double _timeout=0.0);

    /**
     * Clear calibration information from gripper.
     * Allows (and requires) new gripper calibration to be run.
     */
    void clearCalibration();

    /**
     * Power cycle the gripper, removing calibration information in a blocking manner
     * Does not clear errors that could occur during boot
     *
     * @return true/false if success/failure
     */
    bool reboot();

    /**
     * Destructor
     */
    ~Gripper();
};

#endif // __GRIPPER_H__

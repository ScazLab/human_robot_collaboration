#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <mutex>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorProperties.h>

#include "robot_utils/utils.h"

class Gripper
{
private:
    std::string limb;       // Limb of the gripper: left or right

    bool   use_robot;       // Flag to know if we're going to use the robot or not
    bool   first_run;       // Flag to calibrate the gripper at startup if needed

    ros::NodeHandle rnh;      // ROS node handle
    ros::Subscriber sub;      // Subscriber to receive the state of the gripper
    ros::Publisher  pub;      // Publisher for requesting actions to the gripper
    ros::Subscriber sub_prop; // Subscriber to receive the properties of the gripper

    std::mutex mutex;                              // mutex for controlled thread access
    baxter_core_msgs::EndEffectorState state;      // State of the gripper
    baxter_core_msgs::EndEffectorProperties properties; // properties of the gripper

    /**
     * Callback that handles the gripper state messages.
     */
    void gripperCb(const baxter_core_msgs::EndEffectorState &msg);

    /**
     * @brief Calibrates the gripper
     * @details It calibrates the gripper on startup if not already calibrated.
     */
    void calibrate();

    /**
     * Gets the ID of the gripper
     * @return The ID of the gripper
     */
    int get_id();

    /**
     * Raw command call to directly control gripper
     * @param _cmd     string of known gripper commands
     * @param _block   command is blocking or non-blocking
     * @param _timeout timeout in seconds for command evaluation
     * @param _args    parameters and their values in JSON
     */
    void command(std::string _cmd, bool _block=false,
        double _timeout=0.0, std::string _args="");

    /**
     * Closes the gripper
     **/
    void suck();

    /**
     * Opens the gripper
     **/
    void blow();

public:
    /**
     * Constructor of the class
     *
     * @param      _limb either left or right
     * @param _use_robot if to use the robot or not
     **/
    Gripper(std::string _limb, bool _use_robot = true);

    /**
     * Sets the state to the new state, thread-safely
     *
     * @param _state the new state
     */
    void setGripperState(const baxter_core_msgs::EndEffectorState& _state);

    /**
     * Gets the state of the gripper, thread-safely.
     * @return the state of the gripper
     */
    baxter_core_msgs::EndEffectorState getGripperState();

    /**
     * sets and gets the properties of the gripper, thread-safely
     */
    void setGripperProperties(const baxter_core_msgs::EndEffectorProperties& _properties);
    baxter_core_msgs::EndEffectorProperties getGripperProperties();

    bool gripObject();

    bool releaseObject();

    /**
     * Returns a value indicating if the vacuum gripper is enable, so it can be operated.
     * @return true/false if enabled or not.
     **/
    bool is_enabled();

    /**
     * Indicating if the gripper is calibrated.
     * @return True/false if calibrated or not.
     **/
    bool is_calibrated();

    /**
     * Returns a value indicating if the vacuum gripper is ready for gripping.
     * @return True if it is ready, false otherwise.
     **/
    bool is_ready_to_grip();

    /**
     * Returns a value indicating if the gripper is in error state.
     * @return True if the gripper is in error state, false otherwise.
     **/
    bool has_error();

    /**
     * Returns a value indicating if the vacuum gripper is sucking.
     * @return True if it is sucking, false otherwise.
     **/
    bool is_sucking();

    /**
     * Returns a value indicating if the gripper has something attached.
     * @return True if it is gripping an object, false otherwise.
     **/
    bool is_gripping();

    /**
     * Returns the limb the gripper belongs to
     * @return the limb, either "left" or "right"
     **/
    std::string getGripperLimb()  { return limb; };

    void reboot();
    std::string type();
    void gripperCbProp(const baxter_core_msgs::EndEffectorProperties &msg);

    /**
     * Destructor
     */
    ~Gripper();
};

#endif // __GRIPPER_H__

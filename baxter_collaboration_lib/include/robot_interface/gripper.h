#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <pthread.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include "robot_utils/utils.h"

class Gripper
{
private:
    std::string _limb; // Limb of the gripper: left or right

    bool  use_robot;   // Flag to know if we're going to use the robot or not
    bool _first_run;   // Flag to calibrate the gripper at startup if needed

    ros::NodeHandle _nh;            // ROS node handle
    ros::Subscriber _sub_state;     // Subscriber to receive the state of the gripper
    ros::Publisher  _pub_command;   // Publisher for requesting actions to the gripper

    pthread_mutex_t _mutex;

    baxter_core_msgs::EndEffectorState _state;  // State of the gripper

    // Callback that handles the gripper state messages.
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
     * \param limb either left or right limb
     **/
    Gripper(std::string limb, bool _use_robot = true);

    ~Gripper() {};

    bool gripObject();

    bool releaseObject();

    /**
     * Returns a value indicating if the vacuum gripper is enable, so it can be operated.
     * @return True if enabled, false otherwise.
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
     * This function returns the limb the gripper belongs to
     * @return the limb, either "left" or "right"
     **/
    std::string getGripperLimb()  { return _limb; };
};

#endif // __GRIPPER_H__

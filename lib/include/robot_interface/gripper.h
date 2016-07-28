#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <string>

#include <pthread.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

namespace ttt
{

class Gripper
{
private:
    std::string _type; // It identifies the vacuum gripper we are using: left or right

    bool first_run;    // Flag to calibrate grippers at startup if they are not calibrated

    ros::NodeHandle _nh;            // ROS node handle
    ros::Subscriber _sub_state;     // subscriber to receive the messages related to the state of the vacuum gripper
    ros::Publisher  _pub_command;   // publisher for gripping by sucking

    pthread_mutex_t _mutex;
    // It is updated every time a new message with information about the gripper state arrives
    baxter_core_msgs::EndEffectorState _state; 

    // Callback that handles the gripper state messages. 
    void gripperStateCb(const baxter_core_msgs::EndEffectorStateConstPtr& msg); 

    /**
     * @brief Calibrates the gripper
     * @details It calibrates the gripper on startup if not already calibrated.
     */
    void calibrate();

    /**
     * @brief Gets the ID of the gripper
     * @details Gets the ID of the gripper
     * @return The ID of the gripper
     */
    int get_id();

    /**
     * It makes the vacuum gripper suck so, in case it is in contact with an object, it will grip it.
     **/
    void suck();

    /**
     * It makes the vacuum gripper blow air so, in case it has an object graspped, it will release it.
     **/
    void blow();

public:
    /**
     * Constructor of the class
     * \param gripper This indicates if we are using the left or right vacuum gripper. Possible values are just right or left.
     **/
    Gripper(std::string type);

    ~Gripper() {};

    bool gripObject();

    bool releaseObject();

    /**
     * Returns a value indicating if the vacuum gripper is enable, so it can be operated.
     * @return True if it is enabled, false otherwise.
     **/
    bool is_enabled();

    /**
     * Returns a value indicating that the calibration has completed, so it will operate properly.
     * @return True if calibration is done, false otherwise.
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
     * @return True if it is grippring an object, false otherwise.
     **/
    bool is_gripping();

    /**
     * This function returns the limb the gripper belongs to
     * @return the limb, either "left" or "right"
     **/
    std::string get_type()  { return _type; };
};
}

#endif // __GRIPPER_H__

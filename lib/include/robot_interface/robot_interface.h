#ifndef __ROBOT_INTERFACE_H__
#define __ROBOT_INTERFACE_H__

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <pthread.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <baxter_core_msgs/DigitalIOState.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/CollisionAvoidanceState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>

#include "robot_utils/utils.h"
#include "robot_interface/ros_thread.h"
#include "robot_interface/baxter_trac_ik.h"

/**
 * @brief A ROS Thread class
 * @details This class initializes overhead ROS features: subscriber/publishers,
 *          services, callback functions etc.
 */
class RobotInterface
{
protected:
    ros::NodeHandle _n;

private:
    std::string    _name;
    std::string    _limb;       // Limb (either left or right)
    State         _state;       // State of the controller

    bool       _no_robot;       // Flag to know if we're going to use the robot or not
    bool     _use_forces;       // Flag to know if we're going to use the force feedback

    ros::AsyncSpinner spinner;  // AsyncSpinner to handle callbacks

    ros::Publisher  _joint_cmd_pub; // Publisher to control the robot in joint space
    ros::Publisher    _coll_av_pub; // Publisher to suppress collision avoidance behavior

    // IR Sensor
    ros::Subscriber _ir_sub;
    bool              ir_ok;
    float       _curr_range;
    float   _curr_min_range;
    float   _curr_max_range;

    // Inverse Kinematics
    // Default: TRAC IK
    baxterTracIK ik_solver;

    // Alternative IK: baxter-provided IK solver (for the TTT demo)
    bool             _use_trac_ik;
    ros::ServiceClient _ik_client;

    // End-Effector
    ros::Subscriber            _endpt_sub;
    std::vector<double>       _filt_force;
    double                    force_thres;

    geometry_msgs::Point        _curr_pos;
    geometry_msgs::Quaternion   _curr_ori;
    geometry_msgs::Wrench    _curr_wrench;

    // Joint States
    ros::Subscriber         _jntstate_sub;
    sensor_msgs::JointState    _curr_jnts;

    // Mutex to protect joint state variable
    pthread_mutex_t _mutex_jnts;

    // Collision avoidance State
    ros::Subscriber _coll_av_sub;
    bool            is_colliding;

protected:

    // Cuff OK Button (the circular one)
    ros::Subscriber _cuff_sub;

    /*
     * Checks for if the system is ok. To be called inside every thread execution,
     * in order to make it exit gracefully if there is any problem.
     * It also checks for the ROS state.
     * @return true if everything is okay, false otherwise
     */
    bool ok();

    /*
     * checks if end effector has made contact with a token by checking if
     * the range of the infrared sensor has fallen below the threshold value
     *
     * @param      current range values of the IR sensor, and a string
     *            (strict/loose) indicating whether to use a high or low
     *            threshold value
     *
     * return     true if end effector has made contact; false otherwise
     */
    bool hasCollided(std::string mode = "loose");

    /*
     * Checks if the arm has reached its intended pose by comparing
     * the requested and the current poses
     *
     * @param  p     requested position
     * @param  o     requested orientation quaterion
     * @param  mode  (strict/loose) the desired level of precision
     * @return       true/false if success/failure
     */
    bool isPoseReached(double px, double py, double pz,
                       double ox, double oy, double oz, double ow, std::string mode = "loose");

    /*
     * Checks if the arm has reached its intended position by comparing
     * the requested and the current positions
     *
     * @param  p     requested position
     * @param  mode  (strict/loose) the desired level of precision
     * @return       true/false if success/failure
     */
    bool isPositionReached(double px, double py, double pz, std::string mode = "loose");

    /*
     * Checks if the arm has reached its intended orientation by comparing
     * the requested and the current orientations
     *
     * @param  o     requested orientation quaterion
     * @param  mode  (strict/loose) the desired level of precision
     *               (currently not implemented)
     * @return       true/false if success/failure
     */
    bool isOrientationReached(double ox, double oy, double oz, double ow, std::string mode = "loose");

    /*
     * Checks if the arm has reached its intended joint configuration by comparing
     * the requested and the current joint configurations
     *
     * @param  j     requested joint configuration
     * @param  mode  (strict/loose) the desired level of precision
     * @return       true/false if success/failure
     */
    bool isConfigurationReached(baxter_core_msgs::JointCommand joint_cmd, std::string mode = "loose");

    /*
     * Uses built in IK solver to find joint angles solution for desired pose
     *
     * @param     requested PoseStamped
     * @param     array of joint angles solution
     * @return    true/false if success/failure
     */
    bool computeIK(double px, double py, double pz,
                   double ox, double oy, double oz, double ow,
                   std::vector<double>& joint_angles);

    /*
     * Moves arm to the requested pose. This differs from RobotInterface::goToPose because it
     * does not check if the final pose has been reached, but rather it goes in open-loop
     * unitil a fisical contact with the table is reached
     *
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @return true/false if success/failure
     */
    bool goToPoseNoCheck(double px, double py, double pz,
                         double ox, double oy, double oz, double ow);

    bool goToPoseNoCheck(std::vector<double> joint_angles);

    /*
     * Moves arm to the requested pose , and checks if the pose has been achieved
     *
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @param  mode (either loose or strict, it checks for the final desired position)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow,
                  std::string mode="loose", bool disable_coll_av = false);

    /*
     * Sets the joint names of a JointCommand
     *
     * @param    joint_cmd the joint command
     */
    void setJointNames(baxter_core_msgs::JointCommand& joint_cmd);

    /*
     * Sets the joint commands of a JointCommand
     *
     * @param        s0 First  shoulder joint
     * @param        s1 Second shoulder joint
     * @param        e0 First  elbow    joint
     * @param        e1 Second elbow    joint
     * @param        w0 First  wrist    joint
     * @param        w1 Second wrist    joint
     * @param        w2 Third  wrist    joint
     * @param joint_cmd the joint command
     */
    void setJointCommands(double s0, double s1, double e0, double e1,
                                     double w0, double w1, double w2,
                          baxter_core_msgs::JointCommand& joint_cmd);

    /*
     * Detects if the force overcame a set threshold in either one of its three axis
     *
     * @return true/false if the force overcame the threshold
     */
    bool detectForceInteraction();

    /*
     * Waits for a force interaction to occur.
     *
     * @return true when the force interaction occurred
     * @return false if no force interaction occurred after 20s
     */
    bool waitForForceInteraction(double _wait_time = 20.0, bool disable_coll_av = false);

    /*
     * Callback function that sets the current pose to the pose received from
     * the endpoint state topic
     *
     * @param msg the topic message
     */
    void endpointCb(const baxter_core_msgs::EndpointState& msg);

    /*
     * Callback function for the CUFF OK button
     *
     * @param msg the topic message
     */
    void cuffCb(const baxter_core_msgs::DigitalIOState& msg);

    /**
     * Callback for the joint states. Used to seed the
     * inverse kinematics solver
     *
     * @param msg the topic message
     */
    void jointStatesCb(const sensor_msgs::JointState& msg);

    /**
     * Callback for the collision avoidance state. Used to detect
     * if the robot is currently pushed back by the collision avoidance
     * software which is embedded into the Baxter robot and we don't have
     * access to.
     *
     * @param msg the topic message
     */
    void collAvCb(const baxter_core_msgs::CollisionAvoidanceState& msg);

    /*
     * Infrared sensor callback function that sets the current range to the range received
     * from the left hand range state topic
     *
     * @param      The message
     * @return     N/A
     */
    void IRCb(const sensor_msgs::RangeConstPtr& msg);

    /*
     * Filters the forces with a very simple low pass filter
     */
    void filterForces();

    /*
     * hover arm above tokens
     *
     * @param      double indicating requested height of arm (z-axis)
     * return     N/A
     */
    void hoverAboveTokens(double height);

    /**
     * @brief Publishes the desired joint configuration
     * @details Publishes the desired joint configuration in the proper topic
     *
     * @param _cmd The desired joint configuration
     */
    void publish_joint_cmd(baxter_core_msgs::JointCommand _cmd);

    /**
     * @brief Suppresses the collision avoidance for this arm
     * @details Suppresses the collision avoidance. It needs to be called with
     *          a rate of at least 5Hz
     *
     * @param _cmd An empty message to be sent
     */
    void suppressCollisionAv();

public:
    RobotInterface(std::string name, std::string limb,
                   bool no_robot = false, bool use_forces = true, bool use_trac_ik = true);

    virtual ~RobotInterface();

    /*
     * Self-explaining "setters"
     */
    void setName(std::string name) { _name = name; };
    void setState(int state);
    void setTracIK(bool use_trac_ik) { _use_trac_ik = use_trac_ik; };

    bool setIKLimits(KDL::JntArray  ll, KDL::JntArray  ul);

    /*
     * Self-explaining "getters"
     */
    std::string  getName() { return  _name; };
    std::string  getLimb() { return  _limb; };
    State       getState() { return _state; };

    geometry_msgs::Point        getPos()    { return    _curr_pos; };
    geometry_msgs::Quaternion   getOri()    { return    _curr_ori; };
    geometry_msgs::Wrench       getWrench() { return _curr_wrench; };

    bool getIKLimits(KDL::JntArray &ll, KDL::JntArray &ul);

    /*
     * Check availability of the infrared data
    */
    bool    is_ir_ok() { return ir_ok; };

    /*
     * Checks if the robot has to be used or not
     */
    bool is_no_robot() { return _no_robot; };
};

#endif

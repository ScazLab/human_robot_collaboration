#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/JointState.h>

#include "baxter_collaboration_msgs/DoAction.h"

#define SUBSCRIBER_BUFFER 3

#define THREAD_FREQ       100 // [Hz]

// Allowed default states for the system
#define RECOVER  -4
#define KILLED   -3
#define STOPPED  -2
#define ERROR    -1
#define START     0
#define WORKING   1
#define DONE      2

// Both arms
#define ACTION_HOME         baxter_collaboration_msgs::DoAction::Request::ACTION_HOME
#define ACTION_RELEASE      baxter_collaboration_msgs::DoAction::Request::ACTION_RELEASE
#define ACTION_HAND_OVER    "hand_over"
// Only left arm
#define ACTION_GET          "get"
#define ACTION_PASS         "pass"
#define ACTION_GET_PASS     "get_pass"
// Only right arm
#define ACTION_HOLD         "hold"
#define ACTION_START_HOLD   "start_hold"
#define ACTION_END_HOLD     "end_hold"
// Protected action keys used for things that are not real actions
#define LIST_ACTIONS        baxter_collaboration_msgs::DoAction::Request::LIST_ACTIONS
#define LIST_OBJECTS        baxter_collaboration_msgs::DoAction::Request::LIST_OBJECTS

// Response states to send back to the service
#define NO_IR_SENSOR    baxter_collaboration_msgs::DoAction::Response::NO_IR_SENSOR
#define OBJ_NOT_IN_DB   baxter_collaboration_msgs::DoAction::Response::OBJ_NOT_IN_DB
#define NO_OBJ          baxter_collaboration_msgs::DoAction::Response::NO_OBJ
#define ACT_FAILED      baxter_collaboration_msgs::DoAction::Response::ACT_FAILED
#define ACT_NOT_IN_DB   baxter_collaboration_msgs::DoAction::Response::ACT_NOT_IN_DB
#define ACT_NOT_IMPL    baxter_collaboration_msgs::DoAction::Response::ACT_NOT_IMPL
#define INV_KIN_FAILED  baxter_collaboration_msgs::DoAction::Response::INV_KIN_FAILED

#define Z_HIGH         0.400
#define Z_LOW          0.200

#define ARM_SPEED      0.120    // [m/s]
#define ARM_ROT_SPEED  1.000    // [rad/s] ?

#define FORCE_THRES_R   2.0  // [N]
#define FORCE_THRES_L   2.0  // [N]
#define FORCE_ALPHA     0.2
#define FILTER_EPSILON  1e-6
#define FORCE_FILT_VAR_L 0.001
#define FORCE_FILT_VAR_R 0.01
#define REL_FORCE_THRES_L 65.0
#define REL_FORCE_THRES_R 500.0

#define HORIZONTAL_ORI_L      0.0, 0.70, 0.10, 0.70
#define VERTICAL_ORI_L        0.0,  1.0,  0.0,  0.0

#define HORIZONTAL_ORI_R     -0.590, 0.240, -0.298, 0.711
#define HANDOVER_ORI_R        0.0, 0.7, 0.7, 0.0

#define VERTICAL_ORI_R        0.0, 1.0, 0.0, 0.0

#define POOL_POS_L  -0.05, 0.85, 0.30
#define POOL_ORI_L   -0.7,  0.7, 0.0, 0.0

#define POOL_ORI_R    0.7,  0.7, 0.0, 0.0

#define HOME_POS_L   0.65,  0.45
#define HOME_POS_R   0.65, -0.25

#define EPSILON         1e-8

#define OBJ_NOT_FOUND_NUM_ATTEMPTS  30

/*
 * sets the position of a pose
 *
 * @param     pose, and three floats indicating the 3D position
 *
 * return     N/A
 */
void setPosition(geometry_msgs::Pose& pose, float x, float y, float z);

/*
 * sets the orientation of a pose
 *
 * @param     pose, and four floats indicating the 4D orientation quaternion
 *
 * return     N/A
 */
void setOrientation(geometry_msgs::Pose& pose, float x, float y, float z, float w);

/*
 * converts an integer to a string
 *
 * @param      integer to be converted
 *
 * return     converted string
 */
std::string intToString( const int a );

/**
 * converts a vector of integers to a string
 *
 * @param  _v vector of integers to be converted
 * @return    converted string
 */
std::string vectorOfIntToString(std::vector<int> const& _v);

/*
 * converts an double to a string
 *
 * @param      double to be converted
 *
 * return     converted string
 */
std::string doubleToString( const double a );

/**
 * converts a vector of double to a string
 *
 * @param  _v vector of double to be converted
 * @return    converted string
 */

std::string vectorOfDoubleToString(std::vector<double> const& _v);

/**
 * Norm of a vector
 *
 * @param  vector<double> the 3D point as vector
 * @return                the norm of the vector
 */
double norm(std::vector<double> const& _v);

/**
 * Norm of a geometry_msgs::Point
 *
 * @param  geometry_msgs::Point the 3D point
 * @return                      the norm of the point
 */
double norm(const geometry_msgs::Point & _v);

/**
 * Operator + (sum) between two geometry_msgs::Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   the sum of the two
 */
geometry_msgs::Point operator+ (const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Operator - (difference) between two geometry_msgs:Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   the difference of the two (i.e. a - b)
 */
geometry_msgs::Point operator- (const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Operator == (equality) between two geometry_msgs::Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   true/false if a==b or not
 */
bool                 operator==(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Operator * (addition)       between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to sum a with
 * @return   the element-by-element sum of a with b
 */
geometry_msgs::Point operator+ (const geometry_msgs::Point& a, const double& b);

/**
 * Operator - (subtraction)    between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to subtract a with
 * @return   the element-by-element subtraction of a with b
 */
geometry_msgs::Point operator- (const geometry_msgs::Point& a, const double& b);

/**
 * Operator * (multiplication) between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to multiply a with
 * @return   the element-by-element multiplication of a with b
 */
geometry_msgs::Point operator* (const geometry_msgs::Point& a, const double& b);

/**
 * Operator / (division)       between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to divide a with
 * @return   the element-by-element division of a with b
 */
geometry_msgs::Point operator/ (const geometry_msgs::Point& a, const double& b);

/**
 * Dot, or scalar, product between two geometry_msgs::Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   the dot product
 */
double dot(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Assigns a set of doubles to a quaternion. It is not possible to create the
 * assignment operator as a free function, so I had to resort to this hack
 *
 * @param q The quaternion to assign the doubles to
 * @param x The new x value of the quaternion
 * @param y The new y value of the quaternion
 * @param z The new z value of the quaternion
 * @param w The new w value of the quaternion
 */
void quaternionFromDoubles(geometry_msgs::Quaternion &q,
                           double x, double y, double z, double w);
/**
 * Print function for a geometry_msgs::Point.
 *
 * @return A text description of the Point.
 */
std::string print(geometry_msgs::Point p);

/**
 * Print function for a geometry_msgs::Quaternion.
 *
 * @return A text description of the Quaternion.
 */
std::string print(geometry_msgs::Quaternion q);

/**
 * Print function for a geometry_msgs::Pose.
 *
 * @return A text description of the Pose.
 */
std::string print(geometry_msgs::Pose p);

/**
 * Struct that handles the state of the RobotInterface Class
 */
struct State
{
private:

    int       state;
    ros::Time  time;

public:

    /**
     * Constructor, with default initializations of the state and time
     */
    State(int _s = START, ros::Time _t = ros::Time::now()) : state(_s), time(_t) { };

    /**
     * Sets the state to a new state. Updates the time accordingly.
     *
     * @param _s the new state
     */
    void set(int _s);

    /**
     * Returns the state as an integer
     */
    operator int();

    /**
     * Returns the state as a std::string (i.e. a text description of the state)
     */
    operator std::string();

    /**
     * Returns the state as a ros::Time object (i.e. when the state was last set)
     */
    operator ros::Time();
};

#endif

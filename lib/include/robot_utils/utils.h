#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/JointState.h>

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
#define ACTION_HOME         "home"
#define ACTION_RELEASE      "release"
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
#define PROT_ACTION_LIST    "list_actions"  // list the available actions

#define Z_HIGH         0.400
#define Z_LOW          0.200
#define PICK_UP_SPEED  0.120    // [m/s]

#define FORCE_THRES_R   2.0  // [N]
#define FORCE_THRES_L   2.0  // [N]
#define FORCE_ALPHA     0.3

#define HORIZONTAL_ORI_L      0.0, 0.70, 0.10, 0.70
#define VERTICAL_ORI_L        0.0,  1.0,  0.0,  0.0

#define HORIZONTAL_ORI_R     -0.590, 0.240, -0.298, 0.711
#define HANDOVER_ORI_R        0.0, 0.7, 0.7, 0.0

#define VERTICAL_ORI_R        0.0, 1.0, 0.0, 0.0

#define POOL_POS_L  -0.05, 0.85, 0.30
#define POOL_ORI_L   -0.7,  0.7, 0.0, 0.0

#define HOME_POS_L   0.65,  0.45
#define HOME_POS_R   0.65, -0.25

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
 * Print function.
 *
 * @return A text description of the Point
 */
std::string print(geometry_msgs::Point p);

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
    operator int ();

    /**
     * Returns the state as a std::string (i.e. a text description of the state)
     */
    operator std::string ();

    /**
     * Returns the state as a ros::Time object (i.e. when the state was last set)
     */
    operator ros::Time ();
};

#endif

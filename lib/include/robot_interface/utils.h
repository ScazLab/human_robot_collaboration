#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/JointState.h>

#define WORKING  -3
#define KILLED   -2
#define ERROR    -1
#define START     0
#define REST      1
#define SCANNED   2
#define PICK_UP   3
#define PUT_DOWN  4
#define DONE      5

#define SUBSCRIBER_BUFFER 3

#define ACTION_HOME         "home"
#define ACTION_GET          "get"
#define ACTION_RELEASE      "release"
#define ACTION_PASS         "pass"
#define ACTION_HOLD         "hold"
#define ACTION_HAND_OVER    "hand_over"
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
 * checks if two numbers rounded up to 2 decimal points are within 0.0z
 * (z is specified no.) to each other
 *
 * @param      two floats x and y specifying the numbers to be checked,
 *            and a float z determining the desired accuracy
 *
 * return     true if they are within 0.0z; false otherwise
 */
bool withinXHundredth(float x, float y, float z);

/*
 *
 *
 * @param
 *
 * @return  true/false if success/failure
 */
bool withinThres(float x, float y, float t);

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
 * Struct that handles the state of the RobotInterface Class
 */
struct State {
    int state;
    float time;

    State(int _s, float _t) : state(_s), time(_t) { };

    operator int ();

    operator std::string();
};

struct IK_call
{
    struct IK_req
    {
        geometry_msgs::PoseStamped pose_stamp;
        sensor_msgs::JointState   seed_angles;
    };

    struct IK_res
    {
        sensor_msgs::JointState joints;
        bool                   isValid;
    };

    IK_req req;
    IK_res res;
};

#endif

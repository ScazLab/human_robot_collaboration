#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>

#define WORKING  -2
#define ERROR    -1
#define START     0
#define REST      1
#define SCANNED   2
#define PICK_UP   3
#define PUT_DOWN  4
#define DONE      5

#define SUBSCRIBER_BUFFER 3

#define POS_HIGH        0.400
#define POS_LOW         0.200
#define PICK_UP_SPEED   0.1

#define FORCE_THRES     2.0  // [N]
#define FORCE_ALPHA     0.3

#define ACTION_HOME    "home"
#define ACTION_GET     "get"
#define ACTION_RELEASE "release"
#define ACTION_PASS    "pass"
#define ACTION_HOLD    "hold"

#define HORIZONTAL_ORIENTATION_LEFT_ARM       -0.0155113251266, 0.703354199922, -0.102825501874, 0.703192139041
#define VERTICAL_ORIENTATION_LEFT_ARM          0.712801568376, -0.700942136419, -0.0127158080742, -0.0207931175453

#define HORIZONTAL_ORIENTATION_RIGHT_ARM_OLD  -0.175730746765, 0.67317042445, 0.1902242414, 0.692657940308
#define HORIZONTAL_ORIENTATION_RIGHT_ARM      -0.589631754695, 0.239895010244, -0.298576525167, 0.711081455625

#define VERTICAL_ORIENTATION_RIGHT_ARM        -0.14007673309, 0.989696832226, -0.0103020489473, 0.0277949080985

#define POOL_POSITION_LEFT_ARM  -0.05,  0.85, 0.30
#define HOME_POSITION_LEFT_ARM   0.65,  0.45
#define HOME_POSITION_RIGHT_ARM  0.65, -0.25

/*
 * checks if the arm has completed its intended move by comparing
 * the requested pose and the current pose
 * 
 * @param      requested pose and current pose, and a string (strict/loose)
 *            indicating the desired level of checking accuracy 
 *             
 * return     true if the parameters of the current pose is equal to the 
 *            requested pose; false otherwise 
 */
bool hasPoseCompleted(geometry_msgs::Pose a, geometry_msgs::Pose b, std::string mode);

/*
 * checks if two numbers rounded up to 2 decimal points are within 0.0z (z is specified no.) to each other 
 * 
 * @param      two floats x and y specifying the numbers to be checked,
 *            and a float z determining the desired accuracy
 *             
 * return     true if they are within 0.0z; false otherwise
 */
bool withinXHundredth(float x, float y, float z);

/*
 * checks if two decimal numbers are equal to each other up to z of decimal points
 * 
 * @param      two floats x and y, and a float z specifying the desired accuracy
 *             
 * return     true if they are equal up to z decimal points; false otherwise
 */
bool equalXDP(float x, float y, float z);

/*
 * sets the position of a pose
 * 
 * @param      Pose* pose, and three floats indicating the x-y-z coordinates of a position
 *             
 * return     N/A
 */
void setPosition(geometry_msgs::Pose& pose, float x, float y, float z);

/*
 * sets the orientation of a pose
 * 
 * @param      Pose* pose, and three floats indicating the x-y-z-w coordinates of an orientation
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

struct State {
    int state;
    float time;

    State(int _s, float _t) : state(_s), time(_t) { };

    operator int ()
    {
        return state;
    }
};



#endif

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

#define FORCE_THRES_R   2.0  // [N]
#define FORCE_THRES_L   2.0  // [N]
#define FORCE_ALPHA     0.3

#define ACTION_HOME         "home"
#define ACTION_GET          "get"
#define ACTION_RELEASE      "release"
#define ACTION_PASS         "pass"
#define ACTION_HOLD         "hold"
#define ACTION_HAND_OVER    "hand_over"

#define HORIZONTAL_ORI_L        0.0, 0.70, 0.10, 0.70
#define VERTICAL_ORI_L          0.0,  1.0,  0.0,  0.0

#define HORIZONTAL_ORI_R_OLD  -0.175730746765, 0.67317042445, 0.1902242414, 0.692657940308
#define HORIZONTAL_ORI_R      -0.589631754695, 0.239895010244, -0.298576525167, 0.711081455625

#define VERTICAL_ORI_R        0.0, 1.0, 0.0, 0.0

#define POOL_POS_L  -0.05,  0.85, 0.30
#define HOME_POS_L   0.65,  0.45
#define HOME_POS_R  0.65, -0.25

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

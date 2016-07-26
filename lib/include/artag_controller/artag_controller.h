#ifndef ARTAG_CONTROLLER_H
#define ARTAG_CONTROLLER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>
#include <arm_controller/arm_controller.h>

#define ACTION_HOME    "home"
#define ACTION_GET     "get"
#define ACTION_RELEASE "release"
#define ACTION_PASS    "pass"

#define POS_HIGH        0.400
#define POS_LOW         0.150
#define PICK_UP_SPEED   0.1

#define HORIZONTAL_ORIENTATION_LEFT_ARM     -0.0155113251266, 0.703354199922, -0.102825501874, 0.703192139041

class ARTagController : public ROSThread
{
private:
    double elapsed_time;

    ros::Subscriber _aruco_sub;

    geometry_msgs::Pose _curr_marker_pose;

    std::string action;
    int marker_id;

    void clearMarkerPose();

    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow);

    void ARCallback(const aruco_msgs::MarkerArray& msg);

    void hoverAboveTokens(double height);

    void moveObjectTowardHuman();

    bool pickARTag();

    bool pickObject();

    bool passObject();

protected:
    /*
     * picks up token
     * 
     * param      N/A
     * return     N/A
     */
    void InternalThreadEntry();

public:
    ARTagController(std::string limb);

    bool goHome();

    bool releaseObject();

    void setAction(std::string _action) { action = _action; };

    void setMarkerID(int _id) { marker_id = _id; }

    ~ARTagController();
};

#endif

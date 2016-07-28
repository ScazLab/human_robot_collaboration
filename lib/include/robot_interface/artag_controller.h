#ifndef ARTAG_CONTROLLER_H
#define ARTAG_CONTROLLER_H

#include <aruco_msgs/MarkerArray.h>

#include "arm_controller.h"


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

    bool hoverAboveTokens(double height);

    bool moveObjectTowardHuman();

    bool pickARTag();

    bool pickObject();

    bool passObject();

    bool goHome();

    bool releaseObject();

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

    void setAction(std::string _action) { action = _action; };

    void setMarkerID(int _id) { marker_id = _id; }

    ~ARTagController();
};

#endif

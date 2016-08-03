#ifndef __ARTAG_CONTROLLER_H__
#define __ARTAG_CONTROLLER_H__

#include <aruco_msgs/MarkerArray.h>

#include "robot_interface/ros_thread.h"
#include "robot_interface/gripper.h"

class ARTagController : public ROSThread, public Gripper
{
private:
    std::string action;
    double elapsed_time;
    int marker_id;

    ros::Subscriber _aruco_sub;
    geometry_msgs::Pose _curr_marker_pose;

    void clearMarkerPose();

    /*
     * Moves arm to the requested pose. This differs from ROSThread::goToPose because it 
     * does not check if the final pose has been reached, but rather it goes in open-loop
     * unitil a fisical contact with the table is reached
     * 
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow);

    void ARCallback(const aruco_msgs::MarkerArray& msg);

    void recoverFromError();

    bool hoverAbovePool();

    bool hoverAboveTable(double height);

    bool moveObjectTowardHuman();

    bool pickARTag();

    bool pickObject();

    bool passObject();

    bool goHome();

protected:
    void InternalThreadEntry();

public:
    ARTagController(std::string limb);

    void setAction(std::string _action) { action = _action; };

    void setMarkerID(int _id) { marker_id = _id; };

    ~ARTagController();
};

#endif

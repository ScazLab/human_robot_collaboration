#ifndef __ARTAG_CTRL_H__
#define __ARTAG_CTRL_H__

#include <aruco_msgs/MarkerArray.h>

#include "robot_interface/arm_ctrl.h"

class ARTagCtrl : public ArmCtrl
{
private:
    double elapsed_time;

    ros::Subscriber _aruco_sub;
    geometry_msgs::Point        _curr_marker_pos;
    geometry_msgs::Quaternion   _curr_marker_ori;

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

    void ArucoCb(const aruco_msgs::MarkerArray& msg);

    bool hoverAbovePool();

    bool moveObjectTowardHuman();

    bool pickARTag();

    bool pickObject();

    bool passObject();

    bool handOver();

    geometry_msgs::Quaternion computeRotation();

protected:
    void InternalThreadEntry();

public:
    ARTagCtrl(std::string _name, std::string _limb);

    ~ARTagCtrl();
};

#endif

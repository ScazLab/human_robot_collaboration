#ifndef __CARTESIAN_ESTIMATOR_CLIENT__
#define __CARTESIAN_ESTIMATOR_CLIENT__

#include <ros/ros.h>
#include <ros/console.h>

#include <baxter_collaboration/ObjectsArray.h>

#include "robot_utils/utils.h"

class CartesianEstimatorClient
{
private:
    ros::NodeHandle _nh;

    std::string _limb; // Limb of the gripper: left or right

    // Subscriber to the CartesianEstimator detector,
    // plus some flags related to it
    ros::Subscriber _cartest_sub;
    bool              cartest_ok;
    bool          object_found;

    int          object_id;

    // Marker position and orientation
    geometry_msgs::Point        _curr_object_pos;
    geometry_msgs::Quaternion   _curr_object_ori;

protected:

    /**
     * Clears the object pose to reset its state internally
     */
    void clearMarkerPose();

    /**
     * Callback function for the CartesianEstimator topic
     * @param msg the topic message
     */
    void ObjectCb(const baxter_collaboration::ObjectsArray& msg);

    /**
     * Waits for useful data coming from CartesianEstimator
     * @return true/false if success/failure
     */
    bool waitForARucoData();

    /*
     * Check availability of the CartesianEstimator data
    */
    bool is_cartesian_estimator_ok() { return cartest_ok; };

    /* Self-explaining "setters" */
    void setMarkerID(int _id)            { object_id =     _id; };

    /* Self-explaining "getters" */
    geometry_msgs::Point      getMarkerPos() { return _curr_object_pos; };
    geometry_msgs::Quaternion getMarkerOri() { return _curr_object_ori; };

    std::string getCartesianEstimatorLimb() { return     _limb; };
    int         getMarkerID()  { return object_id; };

public:
    CartesianEstimatorClient(std::string name, std::string limb);
    ~CartesianEstimatorClient();

};

#endif

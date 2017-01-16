#ifndef __CARTESIAN_ESTIMATOR_CLIENT__
#define __CARTESIAN_ESTIMATOR_CLIENT__

#include <ros/ros.h>
#include <ros/console.h>

#include <baxter_collaboration/ObjectsArray.h>

#include "robot_utils/utils.h"

class CartesianEstimatorClient
{
private:
    ros::NodeHandle  nh;

    std::string    limb; // Limb of the gripper: left or right

    // Subscriber to the CartesianEstimator detector,
    // plus some flags related to it
    ros::Subscriber  cartest_sub;
    bool              cartest_ok;
    bool            object_found;

    std::string      object_name;

    // Object position and orientation
    geometry_msgs::Point        curr_object_pos;
    geometry_msgs::Quaternion   curr_object_ori;

protected:

    /**
     * Clears the object pose to reset its state internally
     */
    void clearObjectPose();

    /**
     * Callback function for the CartesianEstimator topic
     * @param msg the topic message
     */
    void ObjectCb(const baxter_collaboration::ObjectsArray& _msg);

    /**
     * Waits for useful data coming from CartesianEstimator
     * @return true/false if success/failure
     */
    bool waitForCartEstData();

    /*
     * Check availability of the CartesianEstimator data
    */
    bool is_cartesian_estimator_ok() { return cartest_ok; };

    /* Self-explaining "setters" */
    void setObjectName(std::string _name)    { object_name =     _name; };

    /* Self-explaining "getters" */
    geometry_msgs::Point      getObjectPos() { return curr_object_pos; };
    geometry_msgs::Quaternion getObjectOri() { return curr_object_ori; };

    std::string getCartesianEstimatorLimb() { return        limb; };
    std::string getObjectName()             { return object_name; };

public:
    CartesianEstimatorClient(std::string _name, std::string _limb);
    ~CartesianEstimatorClient();

};

#endif

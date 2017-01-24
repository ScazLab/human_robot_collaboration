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

    // Subscriber to the CartesianEstimator detector
    ros::Subscriber  cartest_sub;

    // List of available objects
    std::vector<std::string> available_objects;

    // Bool to check the CartesianEstimator is fine or not
    bool         cartest_ok;

    // Bool to check if the object was found or not
    bool       object_found;

    // Name of the object to detect
    std::string object_name;

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
     * Waits to get feedback from CartesianEstimator
     * @return true/false if success/failure
     */
    bool waitForCartEstOK();

    /**
     * Waits for useful data coming from CartesianEstimator
     * @return true/false if success/failure
     */
    bool waitForCartEstData();

    /*
     * Check availability of the CartesianEstimator data
    */
    bool is_cartesian_estimator_ok() { return cartest_ok; };

    /* SETTERS */
    void setObjectName(std::string _name)    { object_name =     _name; };

    /* GETTERS */
    geometry_msgs::Point      getObjectPos() { return curr_object_pos; };
    geometry_msgs::Quaternion getObjectOri() { return curr_object_ori; };

    std::string getCartesianEstimatorLimb() { return        limb; };
    std::string getObjectName()             { return object_name; };

    /**
     * Returns a list of available markers
     * @return a list of available markers
     */
    std::vector<std::string> getAvailableObjects() { return available_objects; };

    /**
     * Looks if a set of markers is present among those available.
     * @return the subset of available markers among those avilable
     */
    std::vector<std::string> getAvailableObjects(std::vector<std::string> _objects);

public:
    CartesianEstimatorClient(std::string _name, std::string _limb);
    ~CartesianEstimatorClient();

};

#endif

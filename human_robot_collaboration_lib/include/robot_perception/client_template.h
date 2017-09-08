/**
 * Copyright (C) 2017 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2.1 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#ifndef __CLIENT_TEMPLATE__
#define __CLIENT_TEMPLATE__

#include <ros/ros.h>
#include <ros/console.h>

#include "robot_utils/utils.h"

/**
 * Base class for deriving a generic perception client that reads information from a
 * variety of sources. Needs to be specialized. ARucoClient and CartesianEstimatorClient
 * are two specializations of the ClientTemplate.
 */
template<typename T> class ClientTemplate
{
protected:
    ros::NodeHandle ctnh; // NodeHandle to handle ros stuff
    std::string     limb; // Limb of the gripper: left or right
    ros::Subscriber  sub; // Subscriber to the topic that sends perception information

    bool         is_ok; // Bool to check if the Client is fine or not
    bool objects_found; // Bool to check if there are any objects detected
    bool  object_found; // Bool to check if the selected object was found or not

    // Object position and orientation
    geometry_msgs::Point        curr_object_pos;
    geometry_msgs::Quaternion   curr_object_ori;

    std::vector<T> available_objects; // List of available objects
    T                      object_id; // ID of the object to detect

    /**
     * Resets the cartesian estimator state in order to wait for
     * fresh, new data from the topic
     */
    void reset()
    {
        is_ok = false;
    };

    /**
     * Clears the object found flag to reset its state internally
     * and wait for fresh, new data
     */
    void clearObjFound()
    {
        object_found = false;
    };

    /**
     * Clears the objects_found flag to reset its state internally
     * and wait for fresh, new data
     */
    void clearObjsFound()
    {
        objects_found = false;

        available_objects.clear();
    };

protected:
    // Print level to be used throughout the code
    int ct_print_level;

    /**
     * Waits to get feedback from the perception node.
     *
     * @return true/false if success/failure
     */
    bool waitForOK()
    {
        reset();

        int cnt=0;
        ros::Rate r(10);

        while (!is_ok)
        {
            ROS_WARN_COND(cnt>1, "No callback from perception. Is perception running?");
            ++cnt;

            if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
            {
                ROS_ERROR("No callback from perception! Stopping.");
                return false;
            }

            r.sleep();
        }

        return true;
    };

    /**
     * Waits to see if there are any objects detected
     *
     * @return true/false if success/failure
     */
    bool waitForObjsFound()
    {
        clearObjsFound();

        int cnt=0;
        ros::Rate r(10);

        while (!objects_found)
        {
            ROS_WARN_COND(cnt>1, "Objects not found. Are there any the objects there?");
            ++cnt;

            if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
            {
                ROS_ERROR("Objects not found! Stopping.");
                return false;
            }

            r.sleep();
        }

        return true;
    };

    /**
     * Waits to see if the desired object has been detected
     *
     * @return true/false if success/failure
     */
    bool waitForObjFound()
    {
        clearObjFound();

        int cnt=0;
        ros::Rate r(10);

        while (!object_found)
        {
            ROS_WARN_COND(cnt>0, "Object not found. Is the object there?");
            ++cnt;

            if (cnt == OBJ_NOT_FOUND_NUM_ATTEMPTS)
            {
                ROS_ERROR("Object not found! Stopping.");
                return false;
            }

            r.sleep();
        }

        return true;
    };

    /**
     * Waits for useful data coming from the perception node. It performs
     * all the wait* functions declared above (i.e. waitForOK(),
     * waitForObjsFound() and waitForObjFound())
     * @return true/false if success/failure
     */
    bool waitForData()
    {
        ROS_INFO("[%s] Waiting for data from perception..", getClientLimb().c_str());

        if (!waitForOK())        return false;
        if (!waitForObjsFound()) return false;
        if (!waitForObjFound())  return false;

        return true;
    };

    /*
     * Check status of the client.
     *
     * @return true/false if feedback from the perception is received
    */
    bool isOK() { return   is_ok; };

    /* GETTERS */
    geometry_msgs::Point      getObjectPos() { return curr_object_pos; };
    geometry_msgs::Quaternion getObjectOri() { return curr_object_ori; };

    std::string getClientLimb() { return      limb; };
    T             getObjectID() { return object_id; };

    /* SETTERS */
    void setObjectID(T _id) { object_id = _id; };

    /**
     * Returns a list of available markers
     * @return a list of available markers
     */
    std::vector<T> getAvailableObjects() { return available_objects; };

    /**
     * Looks if a set of markers is present among those available.
     * @return the subset of available markers among those available
     */
    std::vector<T> getAvailableObjects(std::vector<T> _objects)
    {
        std::vector<T> res;

        for (size_t i = 0; i < _objects.size(); ++i)
        {
            if(std::find(available_objects.begin(), available_objects.end(),
                                    _objects[i]) != available_objects.end())
            {
                /* available_objects contains _objects[i] */
                res.push_back(_objects[i]);
            }
        }

        return res;
    };

public:
    /**
     * Constructor
     */
    ClientTemplate(std::string _name, std::string _limb) :
                   ctnh(_name), limb(_limb), is_ok(false),
                   objects_found(false), object_found(false), ct_print_level(0)
    {
        ctnh.param<int> ("/print_level", ct_print_level, 0);
    };

    /**
     * Destructor
     */
    ~ClientTemplate() { };

};

#endif

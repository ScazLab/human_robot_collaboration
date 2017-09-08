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

#ifndef __ARTAG_CTRL_H__
#define __ARTAG_CTRL_H__

#include <stdlib.h>
#include <time.h>

#include <aruco_msgs/MarkerArray.h>

#include "robot_interface/arm_ctrl.h"
#include "robot_perception/perception_client_impl.h"

class ARTagCtrl : public ArmCtrl, public ARucoClient
{
private:
    /**
     * Picks up the selected object by using ARuco's info on the tag
     *
     * @return true/false if success/failure
     */
    virtual bool pickARTag();

    /**
     * [prepare4HandOver description]
     * @return true/false if success/failure
     */
    bool prepare4HandOver();

    /**
     * [waitForOtherArm description]
     * @param  _wait_time      time to wait
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool waitForOtherArm(double _wait_time = 60.0, bool disable_coll_av = false);

protected:

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

    /**
     * [getObject description]
     * @return true/false if success/failure
     */
    virtual bool getObject();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    virtual bool passObject();

    /**
     * [getPassObject description]
     * @return true/false if success/failure
     */
    bool getPassObject();

    /**
     * [handOver description]
     * @return true/false if success/failure
     */
    bool handOver();

    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [recoverRelease description]
     * @return true/false if success/failure
     */
    bool recoverGet();

    /**
     * [recoverRelease description]
     * @return true/false if success/failure
     */
    bool recoverRelease();

    /**
     * Determines if a contact occurred by reading the IR sensor and looking for
     * eventual squish events. Since the SDK does not allow for setting custom squish params,
     * the latter can often fail so there is a check that prevents the end-effector from going
     * too low if this happens.
     *
     * @return true/false if success/failure
     */
    virtual bool determineContactCondition();

    /**
     * Computes object-specific (and pose-specific) offsets in order for the robot
     * to grip the object not in the center of its coordinate system but where
     * it is most convenient for the gripper
     *
     * @param _x_offs The x offset
     * @param _y_offs The y offset
     *
     * @return true/false if success/failure
     */
    virtual bool computeOffsets(double &_x_offs, double &_y_offs);

    /**
     * Computes action-specific orientation in order for the robot to be able to
     * be transparent with respect to different actions in different poses
     *
     * @param _ori The desired orientation as a quaternion
     * @return true/false if success/failure
     */
    virtual bool computeOrientation(geometry_msgs::Quaternion &_ori);

    /**
     * Chooses the object to act upon according to some rule. This method
     * needs to be specialized in any derived class because it is dependent
     * on the type of action and the type of sensory capabilities available.
     *
     * @param _objs The list of IDs of objects to choose from
     * @return      the ID of the chosen object (by default the ID of the
     *              first object will be chosen)
     */
    int chooseObjectID(std::vector<int> _objs);

public:
    /**
     * Constructor
     */
    ARTagCtrl(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~ARTagCtrl();

    void setObjectID(int _obj);
};

#endif

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

#ifndef __TOOL_PICKER_H__
#define __TOOL_PICKER_H__

#include <stdlib.h>
#include <time.h>

#include <flatpack_furniture/hold_ctrl.h>
#include <robot_perception/perception_client_impl.h>

class ToolPicker : public ArmCtrl, public PerceptionClientImpl
{
private:
    std::vector<double> squish_thresholds;

    /**
     * Retrieves an object from the pool of objects
     *
     * @return true/false if success/failure
     */
    bool getObject();

    /**
     * Passes an object to the human (or places it onto the workspace)
     *
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * Cleans up the selected obejct from the table by using the cartesian estimator's info
     * @return true/false if success/failure
     */
    bool cleanUpObject();

    /**
     * Picks up the selected object by using the cartesian estimator's info
     * @return true/false if success/failure
     */
    bool pickUpObject();

    /**
     * Determines if a contact occurred by reading the IR sensor and looking for
     * eventual squish events. Since the SDK does not allow for setting custom squish params,
     * the latter can often fail so there is a check that prevents the end-effector from going
     * too low if this happens.
     * @return true/false if success/failure
     */
    bool determineContactCondition();

    /**
     * Computes object-specific (and pose-specific) offsets in order for the robot
     * to grip the object not in the center of its coordinate system but where
     * it is most convenient for the gripper
     *
     * @param x_offs The x offset
     * @param y_offs The y offset
     *
     * @return true/false if success/failure
     */
    bool computeOffsets(double &x_offs, double &y_offs);

    /**
     * Computes action-specific orientation in order for the robot to be able to
     * be transparent with respect to different actions in different poses
     *
     * @param _ori The desired orientation as a quaternion
     * @return true/false if success/failure
     */
    bool computeOrientation(geometry_msgs::Quaternion &_q);

    /**
     * Stores initial squish thresholds to squish_thresholds, then
     * reduces and rewrites squish thresholds to aid in picking up tools
     */
    void reduceSquish();

    /**
     * Resets squish thresholds to original values stored in squish_thresholds
     */
    void resetSquish();

protected:

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

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
    ToolPicker(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~ToolPicker();

    /* SETTERS */
    void setObjectID(int _obj);
};

#endif

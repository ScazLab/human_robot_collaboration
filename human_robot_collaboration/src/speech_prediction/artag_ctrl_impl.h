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

#ifndef __ARTAG_CTRL_IMPL_SPEECH_PREDICTION_H__
#define __ARTAG_CTRL_IMPL_SPEECH_PREDICTION_H__

#include "flatpack_furniture/artag_ctrl.h"

/**
 * This class performs hold actions plus ARuco recognition and pick up.
 * It is not directly inheriting from HoldCtrl to avoid diamond inheritance w.r.t. ArmCtrl.
 */
class ARTagCtrlImpl : public ARTagCtrl
{
private:
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
    bool computeOffsets(double &_x_offs, double &_y_offs)
    {
        if (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
        {
            if      (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) ==          "foot")
            {

            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "bracket_front")
            {

            }
        }
        else
        {
            ROS_ERROR("State is neither ACTION_GET or ACTION_GET_PASS!");
            return false;
        }

        return true;
    }

public:
    /**
     * Constructor
     */
    ARTagCtrlImpl(std::string _name, std::string _limb, bool _use_robot = true) :
                  ARTagCtrl(_name, _limb, _use_robot)
    {
        setHomeConfiguration();
        setArmSpeed(getArmSpeed() / 1.3);

        insertAction(std::string(ACTION_HOLD) + "_leg", static_cast<f_action>(&ARTagCtrlImpl::holdObject));
        insertAction(std::string(ACTION_HOLD) + "_top", static_cast<f_action>(&ARTagCtrlImpl::holdObject));

        printActionDB();

        if (not _use_robot) return;

        if (!callAction(ACTION_HOME)) setState(ERROR);
    };

    /**
     * Destructor
     */
    ~ARTagCtrlImpl() { };
};

#endif

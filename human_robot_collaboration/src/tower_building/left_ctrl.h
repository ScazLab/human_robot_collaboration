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

#ifndef __LEFT_CTRL_TOWER_BUILDING_H__
#define __LEFT_CTRL_TOWER_BUILDING_H__

#include "robot_interface/arm_perception_ctrl.h"

class LeftCtrl : public ArmPerceptionCtrl
{
private:
    /**
     * Recovers from the getPass action
     *
     * @return true/false if success/failure
     */
    bool recoverGetPass()
    {
        if (!homePoseStrict()) return false;

        if (getPrevAction() == ACTION_GET)
        {
            if (!moveArm("left", 0.1)) return false;
            if (!moveArm("down", 0.3)) return false;
            if (!open())               return false;
            if (!homePoseStrict()) return false;
        }
        return true;
    };

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError()
    {
        if (getInternalRecovery() == true)
        {
            setState(RECOVER);
            recoverGetPass();
            setState(ERROR);
        }
    };

public:
    /**
     * Constructor
     */
    LeftCtrl(std::string _name, std::string _limb, bool _use_robot = true) :
                               ArmPerceptionCtrl(_name, _limb, _use_robot)
    {
        setHomeConfiguration();

        setState(START);

        insertAction("recover_"+std::string(ACTION_GET_PASS),
                     static_cast<f_action>(&LeftCtrl::recoverGetPass));

        printActionDB();

        if (not _use_robot) return;

        if (!callAction(ACTION_HOME)) setState(ERROR);
    };

    /**
     * Destructor
     */
    ~LeftCtrl() { };
};

#endif


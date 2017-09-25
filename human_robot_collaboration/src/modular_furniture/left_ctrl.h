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

#ifndef __LEFT_CTRL_MODULAR_FURNITURE_H__
#define __LEFT_CTRL_MODULAR_FURNITURE_H__

#include "robot_interface/arm_perception_ctrl.h"

class LeftCtrl : public ArmPerceptionCtrl
{
protected:
    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError()
    {
        if (getInternalRecovery() == true)
        {
            ROS_INFO_COND(print_level>=1, "[%s] Recovering from errror..", getLimb().c_str());

            if (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
            {
                recoverGet();
            }
            else
            {
                goHome();
            }
        }
    }

    /**
     * Passes an object to the human (or places it onto the workspace)
     *
     * @return true/false if success/failure
     */
    bool passObject()
    {
        if (getPrevAction() != ACTION_GET)     return false;

        bool human = true;
        if (!moveObjectToPassPosition(human))  return false;

        ros::Duration(0.5).sleep();
        if (!waitForUserCuffUpperFb())         return false;

        std::string object_name = getObjectNameFromDB(ClientTemplate<int>::getObjectID());
        if (object_name != "seat" &&
            object_name != "chair_back" &&
            object_name != "table_top")
        {
            if (!goToPose(0.50, 0.93, 0.2, POOL_ORI_L)) return false;
            ros::Duration(0.22).sleep();
        }

        if (!open())                           return false;
        if (!homePoseStrict())                 return false;

        return true;
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

        removeAction(ACTION_HOLD);
        removeAction(ACTION_PASS);
        insertAction(ACTION_PASS, static_cast<f_action>(&LeftCtrl::passObject));

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

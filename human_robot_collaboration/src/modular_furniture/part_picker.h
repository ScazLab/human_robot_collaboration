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

#ifndef __PART_PICKER_H__
#define __PART_PICKER_H__

#include <flatpack_furniture/artag_ctrl.h>

class PartPicker : public ARTagCtrl
{
protected:
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

        if (ClientTemplate<int>::getObjectID() != 200)
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
    PartPicker(std::string _name, std::string _limb, bool _use_robot = true) :
                                         ARTagCtrl(_name, _limb, _use_robot)
    {
        setHomeConfiguration();

        removeAction(ACTION_PASS);
        insertAction(ACTION_PASS, static_cast<f_action>(&PartPicker::passObject));

        printActionDB();

        if (not _use_robot) return;

        if (!callAction(ACTION_HOME)) setState(ERROR);
    };

    /**
     * Destructor
     */
    ~PartPicker() { };
};

#endif

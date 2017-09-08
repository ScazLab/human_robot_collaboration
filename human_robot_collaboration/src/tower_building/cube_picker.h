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

#ifndef __CUBE_PICKER_H__
#define __CUBE_PICKER_H__

#include <flatpack_furniture/artag_ctrl.h>

class CubePicker : public ARTagCtrl
{
private:
    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [recoverPickPass description]
     * @return true/false if success/failure
     */
    bool recoverPickPass();

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError();

public:
    /**
     * Constructor
     */
    CubePicker(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~CubePicker();
};

#endif

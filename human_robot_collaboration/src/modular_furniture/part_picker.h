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
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

public:
    /**
     * Constructor
     */
    PartPicker(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~PartPicker();
};

#endif

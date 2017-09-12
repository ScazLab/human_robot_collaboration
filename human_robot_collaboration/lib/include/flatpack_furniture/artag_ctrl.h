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

#include "robot_interface/arm_perception_ctrl.h"

class ARTagCtrl : public ArmPerceptionCtrl
{
private:
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
     * [handOver description]
     * @return true/false if success/failure
     */
    bool handOver();

public:
    /**
     * Constructor
     */
    ARTagCtrl(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~ARTagCtrl();
};

#endif

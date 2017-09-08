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

#ifndef __ARTAG_CTRL_IMPL_FLATPACK_FURNITURE_H__
#define __ARTAG_CTRL_IMPL_FLATPACK_FURNITURE_H__

#include "flatpack_furniture/artag_ctrl.h"

class ARTagCtrlImpl : public ARTagCtrl
{
private:

public:
    /**
     * Constructor
     */
    ARTagCtrlImpl(std::string _name, std::string _limb, bool _use_robot = true) :
                                             ARTagCtrl(_name,_limb, _use_robot)
    {
        setHomeConfiguration();

        insertAction(ACTION_HAND_OVER, static_cast<f_action>(&ARTagCtrlImpl::handOver));

        insertAction("recover_"+std::string(ACTION_RELEASE),
                     static_cast<f_action>(&ARTagCtrlImpl::recoverRelease));

        insertAction("recover_"+std::string(ACTION_GET),
                      static_cast<f_action>(&ARTagCtrlImpl::recoverGet));

        // Not implemented actions throw a ROS_ERROR and return always false:
        insertAction("recover_"+std::string(ACTION_PASS),      &ARTagCtrlImpl::notImplemented);
        insertAction("recover_"+std::string(ACTION_HAND_OVER), &ARTagCtrlImpl::notImplemented);

        printActionDB();

        if (not _use_robot) return;

        if (!callAction(ACTION_HOME)) setState(ERROR);

        // moveArm("up",0.2,"strict");
        // moveArm("down",0.2,"strict");
        // moveArm("right",0.2,"strict");
        // moveArm("left",0.2,"strict");
        // moveArm("forward",0.1,"strict");
        // moveArm("backward",0.2,"strict");
        // moveArm("forward",0.1,"strict");
    };

    /**
     * Destructor
     */
    ~ARTagCtrlImpl() { };
};

#endif

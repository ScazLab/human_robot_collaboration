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

#ifndef __RIGHT_CTRL_SPEECH_PREDICTION_H__
#define __RIGHT_CTRL_SPEECH_PREDICTION_H__

#include "robot_interface/arm_perception_ctrl.h"

class RightCtrl : public ArmPerceptionCtrl
{
protected:
    /**
     * Determines if a contact occurred by reading the IR sensor and looking for
     * eventual squish events. Since the SDK does not allow for setting custom squish params,
     * the latter can often fail so there is a check that prevents the end-effector from going
     * too low if this happens.
     *
     * @return true/false if success/failure
     */
    bool determineContactCondition()
    {
        if (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
        {
            if      (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_2" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_3" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_4"   )
            {
                if (getPos().z < -0.315)
                {
                    ROS_INFO("Object reached!");
                    return true;
                }
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "front_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "front_2" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) ==   "top_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) ==   "top_2" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) ==  "back_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) ==  "back_2"   )
            {
                if (getPos().z < -0.30)
                {
                    ROS_INFO("Object reached!");
                    return true;
                }
            }
            if      (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver_2"   )
            {
                if (getPos().z < -0.325)
                {
                    ROS_INFO("Object reached!");
                    return true;
                }
            }
        }
        return false;
    };

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
            if      (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_2" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_3" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "foot_4"   )
            {
                _y_offs = -0.05222;
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "front_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "front_2"   )
            {
                _x_offs = +0.01091;
                _y_offs = -0.03952;
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "top_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "top_2"   )
            {
                _y_offs = -0.04183;
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "back_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "back_2"   )
            {
                _x_offs = +0.00295;
                _y_offs = -0.06204;
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver_1" ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver_2"   )
            {
                _y_offs = -0.08195;
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
    RightCtrl(std::string _name, std::string _limb, bool _use_robot = true) :
                                ArmPerceptionCtrl(_name, _limb, _use_robot)
    {
        setHomeConfiguration();
        setArmSpeed(getArmSpeed() / 1.3);

        insertAction(std::string(ACTION_HOLD) + "_leg", static_cast<f_action>(&RightCtrl::holdObject));
        insertAction(std::string(ACTION_HOLD) + "_top", static_cast<f_action>(&RightCtrl::holdObject));

        printActionDB();

        if (not _use_robot) return;

        if (!callAction(ACTION_HOME)) setState(ERROR);
    };

    /**
     * Destructor
     */
    ~RightCtrl() { };
};

#endif

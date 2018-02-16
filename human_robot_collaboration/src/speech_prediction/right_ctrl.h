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
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError()
    {
        if (getInternalRecovery() == true)
        {
            ROS_INFO_COND(print_level>=1, "[%s] Recovering from error..", getLimb().c_str());

            if ((getAction() == ACTION_GET || getAction() == ACTION_GET_PASS) && is_gripping())
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
            std::string object_name = getObjectNameFromDB(ClientTemplate<int>::getObjectID());
            double z_contact = 0.0;

            if      (object_name == "foot_1" || object_name == "foot_2" ||
                     object_name == "foot_3" || object_name == "foot_4" ||
                     object_name == "foot_5" || object_name == "foot_6"   )
            {
                z_contact = -0.327;
            }
            else if (object_name == "front_1" || object_name == "front_2" ||
                     object_name == "front_3" || object_name == "front_4" ||
                     object_name ==   "top_1" || object_name ==   "top_2" ||
                     object_name ==   "top_3" || object_name ==   "top_4"   )
            {
                z_contact = -0.312;
            }
            else if (object_name ==  "back_1" || object_name ==  "back_2")
            {
                z_contact = -0.320;
            }
            else if (object_name == "screwdriver_1"   )
            {
                z_contact = -0.337;
            }
            else
            {
                return ArmPerceptionCtrl::determineContactCondition();
            }

            if (getPos().z < z_contact)
            {
                ROS_INFO("Object reached!");
                return true;
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
            std::string object_name = getObjectNameFromDB(ClientTemplate<int>::getObjectID());

            if      (object_name == "foot_1" || object_name == "foot_2" ||
                     object_name == "foot_3" || object_name == "foot_4" ||
                     object_name == "foot_5" || object_name == "foot_6"   )
            {
                _y_offs = -0.05222;
            }
            else if (object_name == "front_1" || object_name == "front_2" ||
                     object_name == "front_3" || object_name == "front_4"   )
            {
                _x_offs = +0.01091;
                _y_offs = -0.03952;
            }
            else if (object_name == "top_1" || object_name == "top_2"   )
            {
                _x_offs = +0.01300;
                _y_offs = -0.05500;
            }
            else if (object_name == "back_1" || object_name == "back_2"   )
            {
                _x_offs = +0.00295;
                _y_offs = -0.06204;
            }
            else if (object_name == "screwdriver_1")
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

        removeAction(std::string(ACTION_HOLD));

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

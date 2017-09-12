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

#ifndef __TOOL_PICKER_H__
#define __TOOL_PICKER_H__

#include <stdlib.h>
#include <time.h>

#include <robot_interface/arm_perception_ctrl.h>

class ToolPicker : public ArmPerceptionCtrl
{
private:
    /**
     * Moves an object to its final position during pass actions.
     *
     * @param  _human  if the object needs to be taken by an human
     * @return         true/false if success/failure
     */
    bool moveObjectToPassPosition(bool &_human)
    {
        if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box"  ||
            getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
        {
            if (!hoverAboveTable(Z_LOW))    return false;

            if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
            {
                if (!goToPose(0.63, -0.10, -0.14, VERTICAL_ORI_R2)) return false;
            }
            else
            {
                if (!goToPose(0.63, -0.30, -0.14, VERTICAL_ORI_R2)) return false;
            }
        }
        else
        {
            return ArmCtrl::moveObjectToPassPosition(_human);
        }

        return true;
    };


    /**
     * Moves an object to its final position in the pool during cleanup actions.
     *
     * @return         true/false if success/failure
     */
    bool moveObjectToPoolPosition()
    {
        if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver")
        {
            return goToPose( 0.20, -0.85, -0.30, POOL_ORI_R);
        }
        else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
        {
            return goToPose( 0.00, -0.85, -0.25, POOL_ORI_R);
        }
        else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box")
        {
            return goToPose(-0.15, -0.85, -0.25, POOL_ORI_R);
        }

        return false;
    };

    /**
     * Determines if a contact occurred by reading the IR sensor and looking for
     * eventual squish events. Since the SDK does not allow for setting custom squish params,
     * the latter can often fail so there is a check that prevents the end-effector from going
     * too low if this happens.
     * @return true/false if success/failure
     */
    bool determineContactCondition()
    {
        if (hasCollidedIR("strict") || hasCollidedCD())
        {
            if (hasCollidedCD())
            {
                moveArm("up", 0.002);
            }
            ROS_INFO("Collision!");
            return true;
        }
        else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) != "screwdriver")
        {
            if (getAction() == ACTION_CLEANUP)
            {
                if (getPos().z < -0.17)
                {
                    ROS_INFO("Object reached!");
                    return true;
                }
            }
            else if (getAction() == ACTION_GET      ||
                     getAction() == ACTION_GET_PASS)
            {
                if (getPos().z < -0.28)
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
        if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
        {
            if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver")
            {
                _x_offs = +0.010;
                // _y_offs = +0.017;
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box"  ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
            {
                _x_offs = +0.06;
            }
        }
        else if (getAction() == ACTION_CLEANUP)
        {
            if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screwdriver")
            {
                // _x_offs = -0.020;
                _y_offs = -0.010;
            }
            else if (getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "screws_box"  ||
                     getObjectNameFromDB(ClientTemplate<int>::getObjectID()) == "brackets_box")
            {
                _x_offs = +0.020;
                _y_offs = -0.058;
            }
        }
        else
        {
            ROS_ERROR("State is neither ACTION_GET, ACTION_GET_PASS or ACTION_CLEANUP!");
            return false;
        }

        return true;
    };

public:
    /**
     * Constructor
     */
    ToolPicker(std::string _name, std::string _limb,
               bool _use_robot = true) : ArmPerceptionCtrl(_name, _limb, _use_robot)
    {
        setHomeConfiguration();
        setArmSpeed(getArmSpeed() / 1.3);

        setState(START);

        insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
        insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
        insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));
        insertAction(ACTION_CLEANUP,   static_cast<f_action>(&ToolPicker::cleanUpObject));

        removeAction(ACTION_HOLD);

        insertAction(std::string(ACTION_HOLD) + "_leg", static_cast<f_action>(&ToolPicker::holdObject));
        insertAction(std::string(ACTION_HOLD) + "_top", static_cast<f_action>(&ToolPicker::holdObject));

        printActionDB();

        if (not _use_robot) return;

        // reduceSquish();

        if (!callAction(ACTION_HOME)) setState(ERROR);
    };

    /**
     * Destructor
     */
    ~ToolPicker() { };

};

#endif

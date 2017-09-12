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

#ifndef __RIGHT_CTRL_FLATPACK_FURNITURE_H__
#define __RIGHT_CTRL_FLATPACK_FURNITURE_H__

#include "robot_interface/arm_perception_ctrl.h"

class RightCtrl : public ArmPerceptionCtrl
{
private:
    /**
     * [prepare4HandOver description]
     * @return true/false if success/failure
     */
    bool prepare4HandOver()
    {
        ROS_INFO("[%s] Preparing for handover..", getLimb().c_str());

        return goToPose(0.61, 0.15, Z_LOW+0.02, HANDOVER_ORI_R);
    };

    /**
     * [waitForOtherArm description]
     * @param  _wait_time      time to wait
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool waitForOtherArm(double _wait_time = 60.0, bool disable_coll_av = false)
    {
        ros::Time _init = ros::Time::now();

        ros::Rate r(THREAD_FREQ);
        while(RobotInterface::ok())
        {
            if (disable_coll_av)      suppressCollisionAv();

            if (getSubState() == HAND_OVER_DONE)   return true;

            r.sleep();

            if ((ros::Time::now()-_init).toSec() > _wait_time)
            {
                ROS_ERROR("[%s] No feedback from other arm has been received in %gs!",
                                                        getLimb().c_str(), _wait_time);
                return false;
            }
        }
        return false;
    };

    /**
     * [goHoldPose description]
     * @param  height [description]
     * @return        true/false if success/failure
     */
    bool goHandOverPose()
    {
        ROS_INFO("[%s] Going to handover position..", getLimb().c_str());
        return goToPose(0.80, -0.4, 0.24, HORIZONTAL_ORI_R);
    };

    /**
     * [handOver description]
     * @return true/false if success/failure
     */
    bool handOver()
    {
        setSubState(HAND_OVER_START);
        if (!prepare4HandOver())              return false;
        setSubState(HAND_OVER_READY);
        if (!waitForOtherArm(120.0, true))    return false;
        if (!close())                         return false;
        ros::Duration(1.2).sleep();
        if (!goHandOverPose())                return false;
        // ros::Duration(1.0).sleep();
        // if (!waitForForceInteraction(180.0))  return false;
        // if (!open())                          return false;
        // ros::Duration(1.0).sleep();
        // if (!homePoseStrict())         return false;
        setSubState("");

        return true;
    };

protected:
    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration()
    {
        ArmCtrl::setHomeConfiguration("table");
    };

public:
    /**
     * Constructor
     */
    RightCtrl(std::string _name, std::string _limb, bool _use_robot = true) :
                                 ArmPerceptionCtrl(_name,_limb, _use_robot)
    {
        setHomeConfiguration();
        setState(START);

        insertAction(ACTION_START_HOLD, static_cast<f_action>(&RightCtrl::startHold));
        insertAction(ACTION_END_HOLD,   static_cast<f_action>(&RightCtrl::endHold));

        insertAction(ACTION_HAND_OVER,  static_cast<f_action>(&RightCtrl::handOver));

        // Not implemented actions throw a ROS_ERROR and return always false:
        insertAction("recover_"+std::string(ACTION_START_HOLD), &RightCtrl::notImplemented);
        insertAction("recover_"+std::string(ACTION_END_HOLD),   &RightCtrl::notImplemented);
        insertAction("recover_"+std::string(ACTION_HOLD),       &RightCtrl::notImplemented);
        insertAction("recover_"+std::string(ACTION_HAND_OVER),  &RightCtrl::notImplemented);

        printActionDB();

        if (not _use_robot) return;

        if (!callAction(ACTION_HOME)) setState(ERROR);
    };

    /**
     * Callback for the service that lets the two limbs interact
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceOtherLimbCb(human_robot_collaboration_msgs::AskFeedback::Request  &req,
                            human_robot_collaboration_msgs::AskFeedback::Response &res)
    {
        res.success = false;
        if (req.ask == HAND_OVER_READY)
        {
            res.success = false;
            if (getSubState() == HAND_OVER_START) res.reply = HAND_OVER_WAIT;
            if (getSubState() == HAND_OVER_READY)
            {
                setSubState(HAND_OVER_DONE);
                res.reply = getSubState();
            }
        }
        return true;
    };

    /**
     * Destructor
     */
    ~RightCtrl() { };
};

#endif

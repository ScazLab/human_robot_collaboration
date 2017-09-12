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

#ifndef __LEFT_CTRL_FLATPACK_FURNITURE_H__
#define __LEFT_CTRL_FLATPACK_FURNITURE_H__

#include "robot_interface/arm_perception_ctrl.h"

class LeftCtrl : public ArmPerceptionCtrl
{
private:
    /**
     * [prepare4HandOver description]
     * @return true/false if success/failure
     */
    bool prepare4HandOver()
    {
        return moveArm("right", 0.32, "loose", true);
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

        std::string other_limb = getLimb() == "right" ? "left" : "right";

        ROS_INFO("[%s] Waiting for %s arm", getLimb().c_str(), other_limb.c_str());

        ros::ServiceClient _c;
        std::string service_name = "/"+getName()+"/service_"+other_limb+"_to_"+getLimb();
        _c = nh.serviceClient<human_robot_collaboration_msgs::AskFeedback>(service_name);

        human_robot_collaboration_msgs::AskFeedback srv;
        srv.request.ask = HAND_OVER_READY;

        ros::Rate r(THREAD_FREQ);
        while(RobotInterface::ok())
        {
            if (disable_coll_av)      suppressCollisionAv();
            if (!_c.call(srv)) break;

            ROS_DEBUG("[%s] Received: %s ", getLimb().c_str(), srv.response.reply.c_str());

            if (srv.response.reply == HAND_OVER_DONE)
            {
                return true;
            }

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
     * [handOver description]
     * @return true/false if success/failure
     */
    bool handOver()
    {
        if (getPrevAction() != ACTION_GET)  return false;
        if (!prepare4HandOver())            return false;
        ros::Duration(0.2).sleep();
        if (!waitForOtherArm(30.0, true))   return false;
        ros::Duration(0.8).sleep();
        if (!open())                        return false;
        if (!moveArm("up", 0.05))           return false;
        if (!homePoseStrict())              return false;
        setSubState("");

        return true;
    };

public:
    /**
     * Constructor
     */
    LeftCtrl(std::string _name, std::string _limb, bool _use_robot = true) :
                                ArmPerceptionCtrl(_name,_limb, _use_robot)
    {
        setHomeConfiguration();
        setState(START);

        insertAction(ACTION_HAND_OVER, static_cast<f_action>(&LeftCtrl::handOver));

        insertAction("recover_"+std::string(ACTION_RELEASE),
                     static_cast<f_action>(&LeftCtrl::recoverRelease));

        insertAction("recover_"+std::string(ACTION_GET),
                      static_cast<f_action>(&LeftCtrl::recoverGet));

        // Not implemented actions throw a ROS_ERROR and return always false:
        insertAction("recover_"+std::string(ACTION_PASS),      &LeftCtrl::notImplemented);
        insertAction("recover_"+std::string(ACTION_HAND_OVER), &LeftCtrl::notImplemented);

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

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

#ifndef __BAXTER_TRAC_IK_H__
#define __BAXTER_TRAC_IK_H__

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>

class baxterTracIK
{
private:
    std::string       _limb;
    std::string _urdf_param;

    double _timeout;
    double     _eps;
    int  _num_steps;

    TRAC_IK::TRAC_IK *_tracik_solver;
    KDL::Chain _chain;
    KDL::JntArray *_nominal;

public:
    explicit baxterTracIK(std::string limb, bool _use_robot = true);

    ~baxterTracIK();

    KDL::JntArray JointState2JntArray(const sensor_msgs::JointState &js);

    bool perform_ik(baxter_core_msgs::SolvePositionIK &ik_srv);

    bool getKDLLimits(KDL::JntArray &ll, KDL::JntArray &ul);
    bool setKDLLimits(KDL::JntArray  ll, KDL::JntArray  ul);

    void computeFwdKin(KDL::JntArray jointpositions);
};

#endif

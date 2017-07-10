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

#include "robot_utils/particle_thread.h"

/*****************************************************************************/
/*                             ParticleThread                                */
/*****************************************************************************/

ParticleThread::ParticleThread(std::string _name, double _thread_rate,
                               bool _rviz_visualization) : name(_name), r(_thread_rate),
                               is_running(false), is_closing(false),
                               rviz_visualization(_rviz_visualization),
                               start_time(ros::Time::now()), is_particle_set(false),
                               rviz_pub(_name)
{

}

void ParticleThread::internalThread()
{
    start_time = ros::Time::now();

    while(ros::ok() && not is_closing.get())
    {
        // ROS_INFO("Running..");
        Eigen::VectorXd new_pt;

        updateParticle(new_pt);
        setCurrPoint(new_pt);

        // ROS_INFO_STREAM("New particle position: " << getCurrPoint().transpose());

        r.sleep();
    }
}

bool ParticleThread::start()
{
    if (is_particle_set.get() && not is_running.get())
    {
        is_closing.set(false);
        thread     = std::thread(&ParticleThread::internalThread, this);
        is_running.set(true);

        if (rviz_visualization) { rviz_pub.start(); };

        return true;
    }
    else
    {
        return false;
    }
}

bool ParticleThread::stop()
{
    is_closing.set(true);
    is_running.set(false);
    is_particle_set.set(false);

    if (rviz_visualization) { rviz_pub.stop(); };

    if (thread.joinable())
    {
        thread.join();
    }

    is_closing.set(false);

    return true;
}

double ParticleThread::getRate()
{
    return 1/r.expectedCycleTime().toSec();
}

void ParticleThread::setMarker()
{
    Eigen::Vector3d _curr_pt = getCurrPoint();

    geometry_msgs::Pose mrk_pos;
    mrk_pos.position.x = _curr_pt[0];
    mrk_pos.position.y = _curr_pt[1];
    mrk_pos.position.z = _curr_pt[2];

    RVIZMarker curr_mrk(mrk_pos, ColorRGBA(0.0, 1.0, 1.0), 0.03);

    rviz_pub.setMarkers(std::vector<RVIZMarker>{curr_mrk});
}

Eigen::VectorXd ParticleThread::getCurrPoint()
{
    return curr_pt.get();
}

bool ParticleThread::setCurrPoint(const Eigen::VectorXd& _curr_pt)
{
    if (rviz_visualization) { setMarker(); };

    return curr_pt.set(_curr_pt);
}

ParticleThread::~ParticleThread()
{
    stop();
}

/*****************************************************************************/
/*                           ParticleThreadImpl                              */
/*****************************************************************************/
ParticleThreadImpl::ParticleThreadImpl(std::string _name, double _thread_rate,
                                       bool _rviz_visualization) :
                                       ParticleThread(_name, _thread_rate, _rviz_visualization)
{
    is_particle_set.set(true);
}

bool ParticleThreadImpl::updateParticle(Eigen::VectorXd& _new_pt)
{
    _new_pt = Eigen::Vector3d(1.0, 1.0, 1.0);

    return true;
}

/*****************************************************************************/
/*                          LinearPointParticle                              */
/*****************************************************************************/

LinearPointParticle::LinearPointParticle(std::string _name, double _thread_rate,
                                         bool _rviz_visualization) :
                                         ParticleThread(_name, _thread_rate, _rviz_visualization),
                                         speed(double(0.0)), start_pt(Eigen::Vector3d(0.0, 0.0, 0.0)),
                                         des_pt(Eigen::Vector3d(0.0, 0.0, 0.0))
{

}

bool LinearPointParticle::updateParticle(Eigen::VectorXd& _new_pt)
{
    double elap_time = (ros::Time::now() - start_time).toSec();

    Eigen::Vector3d p_sd = des_pt.get() - start_pt.get();

    // We model the particle as a 3D point that moves toward the
    // target with a straight trajectory and constant speed.
    _new_pt = start_pt.get() + p_sd / p_sd.norm() * speed.get() * elap_time;

    Eigen::Vector3d p_cd = des_pt.get() - _new_pt;

    // Check if the current position is overshooting the desired position
    // By checking the sign of the cosine of the angle between p_sd and p_cd
    // This would mean equal to 1 within some small epsilon (1e-8)
    if ((p_sd.dot(p_cd))/(p_sd.norm()*p_cd.norm()) - 1 <  EPSILON &&
        (p_sd.dot(p_cd))/(p_sd.norm()*p_cd.norm()) - 1 > -EPSILON)
    {
        return true;
    }

    _new_pt = des_pt.get();
    return false;
}

void LinearPointParticle::setMarker()
{
    ParticleThread::setMarker();

    Eigen::Vector3d _des_pt = des_pt.get();

    geometry_msgs::Pose mrk_pos;
    mrk_pos.position.x = _des_pt[0];
    mrk_pos.position.y = _des_pt[1];
    mrk_pos.position.z = _des_pt[2];

    RVIZMarker des_mrk(mrk_pos, ColorRGBA(1.0, 1.0, 0.0));

    rviz_pub.push_back(des_mrk);
}

bool LinearPointParticle::setupParticle(const Eigen::Vector3d& _start_pt,
                                        const Eigen::Vector3d&   _des_pt,
                                        double _speed)
{
    start_pt.set(_start_pt);
      des_pt.set(  _des_pt);
       speed.set(   _speed);

    is_particle_set.set(true);

    return true;
}

LinearPointParticle::~LinearPointParticle()
{

}


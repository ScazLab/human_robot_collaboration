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
                               start_time(ros::Time::now()), is_particle_set(false), rviz_pub(_name)
{

}

void ParticleThread::internalThread()
{
    start_time = ros::Time::now();

    while(ros::ok() && not isClosing())
    {
        // ROS_INFO("Running..");
        Eigen::VectorXd new_pt;

        updateParticle(new_pt);
        setCurrPoint(new_pt);

        ROS_INFO_STREAM("New particle position: " << getCurrPoint().transpose());

        r.sleep();
    }
}

bool ParticleThread::start()
{
    if (is_particle_set && not isRunning())
    {
        setIsClosing(false);
        thread     = std::thread(&ParticleThread::internalThread, this);
        setIsRunning(true);

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
    setIsClosing(true);
    setIsRunning(false);
    is_particle_set = false;

    if (rviz_visualization) { rviz_pub.stop(); };

    if (thread.joinable())
    {
        thread.join();
    }

    setIsClosing(false);

    return true;
}

bool ParticleThread::setIsClosing(bool _is_closing)
{
    std::lock_guard<std::mutex> lock(mtx_is_closing);
    is_closing = _is_closing;

    return true;
}

bool ParticleThread::isClosing()
{
    std::lock_guard<std::mutex> lock(mtx_is_closing);
    // ROS_INFO("isClosing %s", is_closing?"TRUE":"FALSE");

    return is_closing;
}

bool ParticleThread::setIsRunning(bool _is_running)
{
    std::lock_guard<std::mutex> lock(mtx_is_running);
    is_running = _is_running;

    return true;
}

double ParticleThread::getRate()
{
    return 1/r.expectedCycleTime().toSec();
}

bool ParticleThread::isRunning()
{
    std::lock_guard<std::mutex> lock(mtx_is_running);
    return is_running;
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
    std::lock_guard<std::mutex> lg(mtx_curr_pt);
    return curr_pt;
}

bool ParticleThread::setCurrPoint(const Eigen::VectorXd& _curr_pt)
{
    if (rviz_visualization) { setMarker(); };

    std::lock_guard<std::mutex> lg(mtx_curr_pt);
    curr_pt = _curr_pt;

    return true;
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
    is_particle_set = true;
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
                                         speed(0.0), start_pt(0.0, 0.0, 0.0),
                                         des_pt(0.0, 0.0, 0.0)
{

}

bool LinearPointParticle::updateParticle(Eigen::VectorXd& _new_pt)
{
    double elap_time = (ros::Time::now() - start_time).toSec();

    Eigen::Vector3d p_sd = getDesPoint() - getStartPoint();

    // We model the particle as a 3D point that moves toward the
    // target with a straight trajectory and constant speed.
    _new_pt = getStartPoint() + p_sd / p_sd.norm() * speed * elap_time;

    Eigen::Vector3d p_cd = getDesPoint() - _new_pt;

    // Check if the current position is overshooting the desired position
    // By checking the sign of the cosine of the angle between p_sd and p_cd
    // This would mean equal to 1 within some small epsilon (1e-8)
    if ((p_sd.dot(p_cd))/(p_sd.norm()*p_cd.norm()) - 1 <  EPSILON &&
        (p_sd.dot(p_cd))/(p_sd.norm()*p_cd.norm()) - 1 > -EPSILON)
    {
        return true;
    }

    _new_pt = getDesPoint();
    return false;
}

void LinearPointParticle::setMarker()
{
    ParticleThread::setMarker();

    Eigen::Vector3d _des_pt = getDesPoint();

    geometry_msgs::Pose mrk_pos;
    mrk_pos.position.x = _des_pt[0];
    mrk_pos.position.y = _des_pt[1];
    mrk_pos.position.z = _des_pt[2];

    RVIZMarker des_mrk(mrk_pos, ColorRGBA(1.0, 1.0, 0.0));

    rviz_pub.push_back(des_mrk);
}

Eigen::VectorXd LinearPointParticle::getStartPoint()
{
    std::lock_guard<std::mutex> lg(mtx_start_pt);
    return start_pt;
}

bool LinearPointParticle::setStartPoint(const Eigen::VectorXd& _start_pt)
{
    std::lock_guard<std::mutex> lg(mtx_start_pt);
    start_pt = _start_pt;

    return true;
}

Eigen::VectorXd LinearPointParticle::getDesPoint()
{
    std::lock_guard<std::mutex> lg(mtx_des_pt);
    return des_pt;
}

bool LinearPointParticle::setDesPoint(const Eigen::VectorXd& _des_pt)
{
    std::lock_guard<std::mutex> lg(mtx_des_pt);
    des_pt = _des_pt;

    return true;
}

bool LinearPointParticle::setupParticle(const Eigen::Vector3d& _start_pt,
                                        const Eigen::Vector3d&   _des_pt,
                                        double _speed)
{
    start_pt = _start_pt;
      des_pt =   _des_pt;
       speed =    _speed;

    is_particle_set = true;

    return true;
}

LinearPointParticle::~LinearPointParticle()
{

}


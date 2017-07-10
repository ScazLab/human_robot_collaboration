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
ParticleThread::ParticleThread(std::string _name, double _thread_rate, bool _rviz_visual) :
                               name(_name), r(_thread_rate), is_running(false), is_closing(false),
                               rviz_visual(_rviz_visual), start_time(ros::Time::now()),
                               is_set(false), rviz_pub(_name)
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
    if (is_set.get() && not is_running.get())
    {
        is_closing.set(false);
        thread = std::thread(&ParticleThread::internalThread, this);
        is_running.set(true);

        if (rviz_visual) { rviz_pub.start(); };

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
    is_set.set(false);

    if (rviz_visual) { rviz_pub.stop(); };

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
    Eigen::VectorXd _curr_pt = getCurrPoint();

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
    bool res = curr_pt.set(_curr_pt);

    if (rviz_visual) { setMarker(); };

    return res;
}

ParticleThread::~ParticleThread()
{
    stop();
}

/*****************************************************************************/
/*                           ParticleThreadImpl                              */
/*****************************************************************************/
ParticleThreadImpl::ParticleThreadImpl(std::string _name, double _thread_rate, bool _rviz_visual) :
                                       ParticleThread(_name, _thread_rate, _rviz_visual)
{
    is_set.set(true);
}

bool ParticleThreadImpl::updateParticle(Eigen::VectorXd& _new_pt)
{
    _new_pt = Eigen::Vector3d(1.0, 1.0, 1.0);

    return true;
}

/*****************************************************************************/
/*                          LinearPointParticle                              */
/*****************************************************************************/
LinearPointParticle::LinearPointParticle(std::string _name, double _thread_rate, bool _rviz_visual) :
                                         ParticleThread(_name, _thread_rate, _rviz_visual),
                                         start_pt(Eigen::Vector3d(0.0, 0.0, 0.0)),
                                         des_pt(Eigen::Vector3d(0.0, 0.0, 0.0)), speed(0.0)
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

    is_set.set(true);

    return true;
}

LinearPointParticle::~LinearPointParticle()
{

}

/*****************************************************************************/
/*                          CircularPointParticle                              */
/*****************************************************************************/
CircularPointParticle::CircularPointParticle(std::string _name, double _thread_rate, bool _rviz_visual) :
                                         ParticleThread(_name, _thread_rate, _rviz_visual),
                                         center(Eigen::Vector3d(0.0, 0.0, 0.0)),
                                         angles(Eigen::Vector2d(0.0, 0.0)), radius(0.0), speed(0.0)
{

}

bool CircularPointParticle::updateParticle(Eigen::VectorXd& _new_pt)
{
    double et = (ros::Time::now() - start_time).toSec(); // Elapsed time

    double p = angles.get()[0]; // Azimuth phi
    double t = angles.get()[1]; // Zenith theta
    double r = radius.get();    // Radius
    double s =  speed.get();    // Speed [rad/s]

    Eigen::Vector3d c(center.get()); // Center of circumference

    // From http://demonstrations.wolfram.com/ParametricEquationOfACircleIn3D/
    Eigen::Vector3d u(-sin(p)       ,        cos(p),      0);
    Eigen::Vector3d n( cos(p)*sin(t), sin(t)*sin(p), cos(t));

    _new_pt = r * cos(s*et) * u + r * sin(s*et) * n.cross(u) + c;

    return true;
}

void CircularPointParticle::setMarker()
{
    ParticleThread::setMarker();

    Eigen::Vector3d _center = center.get();

    geometry_msgs::Pose mrk_pos;
    mrk_pos.position.x = _center[0];
    mrk_pos.position.y = _center[1];
    mrk_pos.position.z = _center[2];

    RVIZMarker des_mrk(mrk_pos, ColorRGBA(1.0, 1.0, 0.0), 0.01);

    rviz_pub.push_back(des_mrk);
}

bool CircularPointParticle::setupParticle(const Eigen::Vector3d& _center,
                                          const Eigen::Vector2d& _angles,
                                          double _radius, double _speed)
{
    center.set(_center);
    angles.set(_angles);
    radius.set(_radius);
     speed.set( _speed);

    is_set.set(true);

    return true;
}

CircularPointParticle::~CircularPointParticle()
{

}

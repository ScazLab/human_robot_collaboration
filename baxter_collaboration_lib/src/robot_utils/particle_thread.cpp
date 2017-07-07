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

ParticleThread::ParticleThread(std::string _name, double _thread_rate, bool _autostart) :
                               nh(_name), spinner(4), name(_name), r(_thread_rate),
                               is_running(false), is_closing(false)
{
    spinner.start();

    if (_autostart == true)
    {
        start();
    }
}

void ParticleThread::internalThread()
{
    // Initial sleep to allow for the constructor
    // of the derived class to finish constructing the object.
    ros::Duration(0.001).sleep();

    while(ros::ok() && not isClosing())
    {
        // ROS_INFO("Running..");

        setCurrPoint(updateParticle());

        r.sleep();
    }
}

bool ParticleThread::start()
{
    if (not isRunning())
    {
        setIsClosing(false);
        thread = std::thread(&ParticleThread::internalThread, this);
        setIsRunning(true);

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

Eigen::Vector3d ParticleThread::getCurrPoint()
{
    std::lock_guard<std::mutex> lg(mtx_curr_pt);
    return curr_pt;
}

bool ParticleThread::setCurrPoint(const Eigen::Vector3d& _curr_pt)
{
    std::lock_guard<std::mutex> lg(mtx_curr_pt);
    curr_pt = _curr_pt;
    return true;
}

ParticleThread::~ParticleThread()
{
    stop();
}

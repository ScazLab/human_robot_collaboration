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

#ifndef __PARTICLE_THREAD_H__
#define __PARTICLE_THREAD_H__

#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include "robot_utils/utils.h"

/**
 * Particle Thread object. Abstract class that implements the thread function and
 * all the related members. Needs to be specialized in any derived class.
 */
class ParticleThread
{
private:
    ros::NodeHandle        nh;
    ros::AsyncSpinner spinner; // AsyncSpinner to handle callbacks

    std::string name; // Name of the object

    std::thread thread; // Thread to update the particle
    ros::Rate        r; // Rate of the thread (in Hz)

    bool           is_running; // Flag to know if the thread has been started
    std::mutex mtx_is_running; // Mutex to protect the thread running flag

    bool           is_closing;  // Flag to close the thread entry function
    std::mutex mtx_is_closing;  // Mutex to protect the thread close flag

    Eigen::Vector3d curr_pt; // Current position of the particle
    std::mutex  mtx_curr_pt; // Mutex to protect access to the marker array

protected:
    /*
     * Function that will be spun out as a thread
     */
    void internalThread();

    /**
     * Updates the particle. To be specialized in derived classes.
     *
     * @return the new updated particle position
     */
    virtual Eigen::Vector3d updateParticle() = 0;

public:
    /**
     * Constructor
     *
     * @param  _name        name of the object
     * @param  _thread_rate period of the timer
     * @param  _autostart   flag to know if to autostart the particle or not
     */
    explicit ParticleThread(std::string _name = "particle_thread",
                            double _thread_rate = THREAD_FREQ,
                            bool _autostart = false);

    /**
     * Starts the particle thread
     *
     * @return true/false if thread created/thread already running
     */
    bool start();

    /**
     * Stops the particle thread
     *
     * @return true/false if success/failure
     */
    bool stop();

    /**
     * Gets if the thread needs to be closed or not
     *
     * @return if the thread needs to be closed or not
     */
    bool isClosing();

    /**
     * Sets if the thread needs to be closed or not
     *
     * @param _is_closing if the thread needs to be closed or not
     * @return            true/false if success/failure
     */
    bool setIsClosing(bool _is_closing);

    /**
     * Gets if the thread is running
     *
     * @return if the thread is running or not
     */
    bool isRunning();

    /**
     * Sets if the thread is running or not
     *
     * @param _is_running if the thread is running or not
     * @return            true/false if success/failure
     */
    bool setIsRunning(bool _is_running);

    /**
     * Gets the rate of the thread
     *
     * @return the rate of the thread
     */
    double getRate();

    /**
     * Gets the current position of the particle
     *
     * @return the current position of the particle
     */
    Eigen::Vector3d getCurrPoint();

    /**
     * Sets the current position of the particle to a new position
     *
     * @param  _curr_pt the new position of the particle
     * @return          true/false if success/failure
     */
    bool setCurrPoint(const Eigen::Vector3d& _curr_pt);

    /**
     * Gets the name of the object
     * @return the name of the object
     */
    std::string     getName() { return      name; };

    /**
     * Destructor
     */
    ~ParticleThread();
};

/**
 * Implementation of ParticleThread object. Does not do much (only for testing purposes)
 */
class ParticleThreadImpl : public ParticleThread
{
protected:
    /**
     * Updates the particle. To be specialized in derived classes.
     */
    Eigen::Vector3d updateParticle() { return Eigen::Vector3d(1.0, 1.0, 1.0); };

public:
    /**
     * Constructor
     *
     * @param  _name        name of the object
     * @param  _thread_rate period of the timer
     * @param  _autostart   flag to know if to autostart the particle or not
     */
    explicit ParticleThreadImpl(std::string _name = "particle_thread_impl",
                                double _thread_rate = THREAD_FREQ,
                                bool _autostart = false) :
                                ParticleThread(_name, _thread_rate, _autostart) {};

    /**
     * Destructor
     */
    ~ParticleThreadImpl() {};
};

#endif

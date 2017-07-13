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

#include "robot_utils/thread_safe.h"
#include "robot_utils/rviz_publisher.h"

/**
 * Particle Thread object. Abstract class that implements the thread function and
 * all the related members. Needs to be specialized in any derived class.
 */
class ParticleThread
{
private:
    std::string   name; // Name of the object
    std::thread thread; // Thread to update the particle
    ros::Rate        r; // Rate of the thread (in Hz)

    ThreadSafe<bool> is_running; // Thread-safe flag to know if the thread has been started
    ThreadSafe<bool> is_closing; // Thread-safe flag to close the thread entry function

    // Current position (or orientation) of the particle (with thread-safe read and write)
    ThreadSafe<Eigen::VectorXd> curr_pt;

    bool rviz_visual; // Flag to know if to publish to rviz or not

protected:
    ros::Time start_time; // When the thread started

    // Thread-safe flag that says if the particle has been already setup.
    // Defaults to false; needs to be specialized in derived classes with proper
    // function that sets it to true (otherwise the thread will not start)
    ThreadSafe<bool> is_set;

    RVIZPublisher  rviz_pub; // Publisher to publish the point to rviz

    /*
     * Function that will be spun out as a thread
     */
    void internalThread();

    /**
     * Updates the particle. To be specialized in derived classes.
     *
     * @param  _new_pt updated point
     * @return         true/false if success/failure
     */
    virtual bool updateParticle(Eigen::VectorXd& _new_pt) = 0;

    /**
     * Sets the current point as a marker for the RVIZPublisher to publish
     */
    virtual void setMarker();

    /**
     * Sets the current position (or orientation) of the particle to a new one
     *
     * @param  _curr_pt the new position (or orientation) of the particle
     * @return          true/false if success/failure
     */
    bool setCurrPoint(const Eigen::VectorXd& _curr_pt);

public:
    /**
     * Constructor
     *
     * @param  _name        name of the object
     * @param  _thread_rate period of the timer
     * @param  _rviz_visual if to publish the current point to rviz as a marker
     */
    explicit ParticleThread(std::string _name   = "particle_thread",
                            double _thread_rate = THREAD_FREQ,
                            bool _rviz_visual   = false);

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
     * Gets the rate of the thread
     * @return the rate of the thread
     */
    double getRate();

    /**
     * Gets the current position (or orientation) of the particle
     * @return the current position (or orientation) of the particle
     */
    Eigen::VectorXd getCurrPoint();

    /**
     * Gets the name of the object
     * @return the name of the object
     */
    std::string getName() { return        name; };

    /**
     * Gets if the thread needs to be closed or not
     * @return if the thread needs to be closed or not
     */
    bool isClosing() { return is_closing.get(); };

    /**
     * Gets if the thread is running
     * @return if the thread is running or not
     */
    bool isRunning() { return is_running.get(); };

    /**
     * Returns if the particle is set or not
     * @return if the particle is set or not
     */
    bool isSet() { return is_set.get(); };

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
     * Updates the particle.
     *
     * @param  _new_pt updated point
     * @return         true/false if success/failure
     */
    bool updateParticle(Eigen::VectorXd& _new_pt);

public:
    /**
     * Constructor
     *
     * @param  _name        name of the object
     * @param  _thread_rate period of the timer
     * @param  _rviz_visual if to publish the current point to rviz as a marker
     */
    explicit ParticleThreadImpl(std::string _name   = "particle_thread_impl",
                                double _thread_rate = THREAD_FREQ,
                                bool _rviz_visual   = false);

    /**
     * Destructor
     */
    ~ParticleThreadImpl() {};
};

/**
 * ParticleThread for a 3D Point following a straight trajectory from start to end.
 */
class LinearPointParticle : public ParticleThread
{
private:
    // Start point of the trajectory (with thread-safe read and write)
    ThreadSafe<Eigen::Vector3d> start_pt;

    // Desired point of the trajectory (with thread-safe read and write)
    ThreadSafe<Eigen::Vector3d>   des_pt;

    // Speed of the trajectory in [m/s] (with thread-safe read and write)
    ThreadSafe<double> speed;

protected:
    /**
     * Updates the particle.
     *
     * @param  _new_pt updated point
     * @return         true/false if success/failure
     */
    bool updateParticle(Eigen::VectorXd& _new_pt);

    /**
     * Sets the current point and the desired target as markers for the RVIZPublisher to publish
     */
    void setMarker();

public:
    /**
     * Constructor
     *
     * @param  _name        name of the object
     * @param  _thread_rate period of the timer
     * @param  _rviz_visual if to publish the current point to rviz as a marker
     */
    explicit LinearPointParticle(std::string _name   = "linear_particle",
                                 double _thread_rate = THREAD_FREQ,
                                 bool _rviz_visual   = false);

    /**
     * Sets the parameters of the particle
     *
     * @param  _start_pt The starting point of the particle
     * @param  _des_pt   The desired (final) point of the particle
     * @param  _speed    The particle speed
     * @return           true/false if success/failure
     */
    bool setupParticle(const Eigen::Vector3d& _start_pt,
                       const Eigen::Vector3d&   _des_pt,
                       double _speed);

    /**
     * Destructor
     */
    ~LinearPointParticle();
};

/**
 * ParticleThread for a 3D Point following a circular trajectory in 3D space.
 * Please be aware that the circular point particle will never stop moving.
 */
class CircularPointParticle : public ParticleThread
{
private:
    // Center of the circumference (with thread-safe read and write)
    ThreadSafe<Eigen::Vector3d> center;

    // Azimuth [-pi; pi] and zenith [0; pi] angles (with thread-safe read and write)
    ThreadSafe<Eigen::Vector2d> angles;

    // Radius of the circumference in meters (with thread-safe read and write)
    ThreadSafe<double> radius;

    // Speed of the trajectory, in [rad/s] (with thread-safe read and write)
    ThreadSafe<double> speed;

protected:
    /**
     * Updates the particle.
     *
     * @param  _new_pt updated point
     * @return         true/false if success/failure
     */
    bool updateParticle(Eigen::VectorXd& _new_pt);

    /**
     * Sets the current point and the desired target as markers for the RVIZPublisher to publish
     */
    void setMarker();

public:
    /**
     * Constructor
     *
     * @param  _name        name of the object
     * @param  _thread_rate period of the timer
     * @param  _rviz_visual if to publish the current point to rviz as a marker
     */
    explicit CircularPointParticle(std::string _name   = "circular_particle",
                                   double _thread_rate = THREAD_FREQ,
                                   bool _rviz_visual   = false);

    /**
     * Sets the parameters of the particle
     *
     * @param  _center The starting point of the particle
     * @param  _angles The azimuth [-pi; pi] and zenith [0; pi] angles
     * @param  _radius The radius of the circumference
     * @param  _speed  The particle speed
     * @return         true/false if success/failure
     */
    bool setupParticle(const Eigen::Vector3d& _center,
                       const Eigen::Vector2d& _angles,
                       double _radius, double _speed);

    /**
     * Destructor
     */
    ~CircularPointParticle();
};

#endif

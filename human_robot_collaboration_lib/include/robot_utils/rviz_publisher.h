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

#ifndef __RVIZ_PUBLISHER_H__
#define __RVIZ_PUBLISHER_H__

#include <mutex>

#include <Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>

#include "robot_utils/utils.h"

/**
 * Struct that wraps an ColorRGBA object from std_msgs with a sane constructor.
 */
struct ColorRGBA
{
    std_msgs::ColorRGBA col;

    /**
     * Default constructor (alpha channel defaults to 1.0)
     */
    ColorRGBA(double _r = 0.5, double _g = 0.5, double _b = 0.5, double _a = 1.0);
};

/**
 * Struct that wraps an RVIZMarker object, and stores its important members
 * See http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html for
 * more information
 */
struct RVIZMarker
{
    // Pose of the object (position + orientation) in case of a single marker, e.g. CUBE
    geometry_msgs::Pose    pose;

    // Points of the objects in case of an array of markers, e.g. a SPHERE_LIST
    std::vector<geometry_msgs::Point> points;

    ColorRGBA   col; // RGBA color to visualize
    double     size; // size in meters
    int        type; // type of object, eg visualization_msgs::Marker::CUBE
    double lifetime; // lifetime of the marker, in seconds

    /**
     * Default Constructor
     */
    RVIZMarker(geometry_msgs::Pose _pose = geometry_msgs::Pose(),
               ColorRGBA _col = ColorRGBA(0.5, 0.5, 0.5, 1.0),
               double _size = 0.05, int _type = visualization_msgs::Marker::CUBE,
               double _lifetime = 20.0);

    /**
     * Constructor that accepts the x, y, z position coordinates of the marker
     */
    RVIZMarker(Eigen::Vector3d _position,
               ColorRGBA _col = ColorRGBA(0.5, 0.5, 0.5, 1.0),
               double _size = 0.05, int _type = visualization_msgs::Marker::CUBE,
               double _lifetime = 20.0);

    /**
     * Constructor that accepts an array of points for complex markers, e.g. SPHERE_LIST
     */
    RVIZMarker(std::vector<geometry_msgs::Point> _points,
               ColorRGBA _col = ColorRGBA(0.5, 0.5, 0.5, 1.0),
               double _size = 0.05, int _type = visualization_msgs::Marker::SPHERE_LIST,
               double _lifetime = 20.0);

    /**
     * Sets the x, y, z coordinates of the marker
     *
     * @param _x x coordinate
     * @param _y y coordinate
     * @param _z z coordinate
     */
    void setPosition(double _x, double _y, double _z);
};

/**
 * Class that wraps an object that publishes an array of RVIZMarkers to RVIZ.
 * In incorporates a timer that keep publishing with a constant rate.
 */
class RVIZPublisher
{
private:
    ros::NodeHandle        nh;
    ros::AsyncSpinner spinner; // AsyncSpinner to handle callbacks

    std::string name; // Name of the object

    std::vector<RVIZMarker> markers; // vector of markers to publish to RVIZ
    std::mutex        markers_mutex; // Mutex to protect access to the marker array
    ros::Publisher         rviz_pub; // Publisher to send markers to RVIZ

    ros::Timer      timer; // Timer to send messages to rviz with a specific period
    double   timer_period; // Timer period
    bool is_timer_created; // Flag to know if the timer has been already created

    /**
     * Callback to publish a vector of markers to RVIZ for visualization
     */
    virtual void publishMarkersCb(const ros::TimerEvent&);

public:
    /**
     * Constructor
     *
     * @param  _name         name of the object
     * @param  _timer_period period of the timer
     */
    explicit RVIZPublisher(std::string _name = "rviz_marker_publisher",
                           double _timer_period = 0.01);

    /**
     * Starts publishing markers to rviz
     *
     * @return true/false if success/failure
     */
    bool start();

    /**
     * Stops publishing markers to rviz. The array of markers is also cleared.
     *
     * @return true/false if success/failure
     */
    bool stop();

    /**
     * Pushes back a marker to the array of markers
     *
     * @param  _mrkr The marker to push back
     */
    void push_back(RVIZMarker _mrkr);

    /**
     * Sets the array of markers. The previous markers will be deleted.
     *
     * @param  _mrkrs The array of markers to set
     * @return        true/false if success/failure
     */
    void setMarkers(std::vector<RVIZMarker> _mrkrs);

    /**
     * Thread-safe access to the array of markers
     *
     * @return the array of markers
     */
    std::vector<RVIZMarker> getMarkers();

    /**
     * Sets a marker in the array with a specific index.
     *
     * @param  _idx  The index to set the marker to
     * @param  _mrkr The marker to set
     * @return       true/false if success/failure
     */
    bool setMarker(size_t _idx, RVIZMarker _mrkr);

    /**
     * Clears the array of markers
     *
     * @return true/false if success/failure
     */
    bool clearMarkers();

    /**
     * Removes a marker with a specific index
     *
     * @param  _idx The index to remove the marker from
     * @return      true/false if success/failure
     */
    bool removeMarker(size_t _idx);

    /**
     * Returns the name of the object
     *
     * @return the name of the object
     */
    std::string getName() { return name; };
};

#endif

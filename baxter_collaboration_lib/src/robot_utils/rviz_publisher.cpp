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

#include "robot_utils/rviz_publisher.h"

using namespace std;

/**************************************************************************/
/*                             ColorRGBA                                  */
/**************************************************************************/

ColorRGBA::ColorRGBA(double _r, double _g, double _b, double _a)
{
    col.r = _r;
    col.g = _g;
    col.b = _b;
    col.a = _a;
};

/**************************************************************************/
/*                             RVIZMarker                                 */
/**************************************************************************/

RVIZMarker::RVIZMarker(geometry_msgs::Pose _pose, ColorRGBA _col,
                       double _size, int _type, double _lifetime) :
                       pose(_pose), col(_col), size(_size),
                       type(_type), lifetime(_lifetime)
{

};

RVIZMarker::RVIZMarker(Eigen::Vector3d _position, ColorRGBA _col,
                       double _size, int _type, double _lifetime) :
                       RVIZMarker(geometry_msgs::Pose(), _col, _size, _type, _lifetime)
{
    setPosition(_position[0], _position[1], _position[2]);
}

RVIZMarker::RVIZMarker(std::vector<geometry_msgs::Point> _points, ColorRGBA _col,
                       double _size, int _type, double _lifetime) :
                       RVIZMarker(geometry_msgs::Pose(), _col, _size, _type, _lifetime)
{
    points = _points;
}

void RVIZMarker::setPosition(double _x, double _y, double _z)
{
    pose.position.x = _x;
    pose.position.y = _y;
    pose.position.z = _z;
}

/**************************************************************************/
/*                            RVIZPublisher                               */
/**************************************************************************/

RVIZPublisher::RVIZPublisher(std::string _name, double _timer_period) :
                             nh(_name), spinner(4), name(_name),
                             timer_period(_timer_period), is_timer_created(false)
{
    rviz_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",
                                                                 SUBSCRIBER_BUFFER, true );

    // ROS_INFO("[%s] RVIZPublisher created. Timer period: %g", getName().c_str(), timer_period);
    spinner.start();
}

void RVIZPublisher::publishMarkersCb(const ros::TimerEvent&)
{
    std::vector<RVIZMarker> _markers = getMarkers();

    // ROS_INFO("[%s] Markers size: %lu", getName().c_str(), markers.size());
    if (_markers.size() > 0)
    {
        visualization_msgs::MarkerArray mrkrs;

        for (size_t i = 0; i < _markers.size(); ++i)
        {
            visualization_msgs::Marker mrkr;

            mrkr.header.frame_id =      "base";
            mrkr.header.stamp    = ros::Time();
            mrkr.ns     =            getName();
            mrkr.id     =               int(i);
            mrkr.type   =     _markers[i].type;
            mrkr.action = visualization_msgs::Marker::ADD;

            if (mrkr.type == visualization_msgs::Marker::LINE_STRIP  ||
                mrkr.type == visualization_msgs::Marker::LINE_LIST   ||
                mrkr.type == visualization_msgs::Marker::CUBE_LIST   ||
                mrkr.type == visualization_msgs::Marker::SPHERE_LIST ||
                mrkr.type == visualization_msgs::Marker::POINTS        )
            {
                for (size_t j = 0; j < _markers[i].points.size(); ++j)
                {
                    mrkr.points.push_back(_markers[i].points[j]);
                }
            }
            else
            {
                mrkr.pose.position.x    = _markers[i].pose.position.x;
                mrkr.pose.position.y    = _markers[i].pose.position.y;
                mrkr.pose.position.z    = _markers[i].pose.position.z;
                mrkr.pose.orientation.x = _markers[i].pose.orientation.x;
                mrkr.pose.orientation.y = _markers[i].pose.orientation.y;
                mrkr.pose.orientation.z = _markers[i].pose.orientation.z;
                mrkr.pose.orientation.w = _markers[i].pose.orientation.w;
            }

            // Custom size of the object
            if (mrkr.type == visualization_msgs::Marker::ARROW)
            {
                mrkr.scale.x = _markers[i].size;
                mrkr.scale.y =             0.01;
                mrkr.scale.z =             0.01;
            }
            else
            {
                mrkr.scale.x = _markers[i].size;
                mrkr.scale.y = _markers[i].size;
                mrkr.scale.z = _markers[i].size;
            }

            mrkr.color.a = _markers[i].col.col.a;
            mrkr.color.r = _markers[i].col.col.r;
            mrkr.color.g = _markers[i].col.col.g;
            mrkr.color.b = _markers[i].col.col.b;

            mrkr.lifetime = ros::Duration(_markers[i].lifetime);

            mrkrs.markers.push_back(mrkr);
        }

        // ROS_INFO("[%s] Publishing", getName().c_str());
        rviz_pub.publish(mrkrs);
    }
}

bool RVIZPublisher::start()
{
    if (not is_timer_created)
    {
        timer = nh.createTimer(ros::Duration(timer_period),
                               &RVIZPublisher::publishMarkersCb, this, false);
        is_timer_created = true;
    }
    else
    {
        timer.start();
    }

    return true;
}

bool RVIZPublisher::stop()
{
    timer.stop();
    std::lock_guard<std::mutex> lg(markers_mutex);
    markers.clear();

    return true;
}

std::vector<RVIZMarker> RVIZPublisher::getMarkers()
{
    std::lock_guard<std::mutex> lg(markers_mutex);
    return markers;
}

void RVIZPublisher::push_back(RVIZMarker _mrkr)
{
    std::lock_guard<std::mutex> lg(markers_mutex);
    markers.push_back(_mrkr);

    // ROS_INFO("[%s] Pushing back. Markers size: %lu",
    //              getName().c_str(), markers.size());
}

void RVIZPublisher::setMarkers(std::vector<RVIZMarker> _mrkrs)
{
    std::lock_guard<std::mutex> lg(markers_mutex);
    markers.clear();
    markers = _mrkrs;
}

bool RVIZPublisher::setMarker(size_t _idx, RVIZMarker _mrkr)
{
    std::lock_guard<std::mutex> lg(markers_mutex);
    if (_idx >= markers.size()) { return false; };

    markers[_idx] = _mrkr;

    return true;
}

bool RVIZPublisher::clearMarkers()
{
    std::lock_guard<std::mutex> lg(markers_mutex);
    markers.clear();

    return true;
}

bool RVIZPublisher::removeMarker(size_t _idx)
{
    std::lock_guard<std::mutex> lg(markers_mutex);
    if (_idx >= markers.size()) { return false; };

    markers.erase(markers.begin() + int(_idx));

    return true;
}

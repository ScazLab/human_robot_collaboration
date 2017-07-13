/**
 * Copyright (C) 2016 Social Robotics Lab, Yale University
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

#include "robot_utils/utils.h"

using namespace std;
using namespace geometry_msgs;

/**************************************************************************/
/*                               Utils                                    */
/**************************************************************************/
void setPosition(Pose& pose, float x, float y, float z)
{
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
}

void setOrientation(Pose& pose, float x, float y, float z, float w)
{
    pose.orientation.x = x;
    pose.orientation.y = y;
    pose.orientation.z = z;
    pose.orientation.w = w;
}

string toString( const int a )
{
    stringstream ss;
    ss << a;
    return ss.str();
}

string toString(vector<int> const& _v)
{
    string      res;
    stringstream ss;

    std::copy(_v.begin(), _v.end(),
              std::ostream_iterator<int>(ss, ", "));

    res = ss.str();
    res = res.substr(0, res.size()-2); // Remove the last ", "
    res = "[" + res + "]";

    return res;
}

string toString( const double a )
{
    stringstream ss;
    ss << a;

    // This handles -0.00, -.0 etc
    if (ss.str() == "-0")    { return "0"; }

    return ss.str();
}

string toString(vector<double> const& _v)
{
	string res = "";

	for(int i = 0, size = _v.size(); i < size; ++i)
	{
		res = res + toString(_v[i]) + ", ";
	}

    // Remove the last ", "
    return "[" + res.substr(0, res.size()-2) + "]";
}

string toString(const geometry_msgs::Pose& _p)
{
    stringstream ss;

    ss <<   "{position:{x: " << toString(_p.position.x)    <<  ", ";
    ss <<              "y: " << toString(_p.position.y)    <<  ", ";
    ss <<              "z: " << toString(_p.position.z)    << "}, ";
    ss << "orientation:{x: " << toString(_p.orientation.x) <<  ", ";
    ss <<              "y: " << toString(_p.orientation.y) <<  ", ";
    ss <<              "z: " << toString(_p.orientation.z) <<  ", ";
    ss <<              "w: " << toString(_p.orientation.w) <<  "}}";

    return ss.str();
}

double norm(vector<double> const& _v)
{
    double accum = 0.0;

    for (size_t i = 0; i < _v.size(); ++i)
    {
        accum += _v[i] * _v[i];
    }
    return sqrt(accum);
}

double norm(const geometry_msgs::Point & _v)
{
    double accum = _v.x * _v.x + _v.y * _v.y + _v.z * _v.z;

    return sqrt(accum);
}

geometry_msgs::Point operator+ (const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    geometry_msgs::Point res;

    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;

    return res;
}

geometry_msgs::Point operator- (const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    geometry_msgs::Point res;

    res.x = a.x - b.x;
    res.y = a.y - b.y;
    res.z = a.z - b.z;

    return res;
}

bool                 operator==(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    if (a.x != b.x)     return false;
    if (a.y != b.y)     return false;
    if (a.z != b.z)     return false;

    return true;
}

geometry_msgs::Point operator+ (const geometry_msgs::Point& a, const double& b)
{
    geometry_msgs::Point res;

    res.x = a.x + b;
    res.y = a.y + b;
    res.z = a.z + b;

    return res;
}

geometry_msgs::Point operator- (const geometry_msgs::Point& a, const double& b)
{
    geometry_msgs::Point res;

    res.x = a.x - b;
    res.y = a.y - b;
    res.z = a.z - b;

    return res;
}

geometry_msgs::Point operator* (const geometry_msgs::Point& a, const double& b)
{
    geometry_msgs::Point res;

    res.x = a.x * b;
    res.y = a.y * b;
    res.z = a.z * b;

    return res;
}

geometry_msgs::Point operator/ (const geometry_msgs::Point& a, const double& b)
{
    geometry_msgs::Point res;

    res.x = a.x / b;
    res.y = a.y / b;
    res.z = a.z / b;

    return res;
}

double dot(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    //We don't check for the size because a geometry_msgs::Point has fixed size
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

std::string print(geometry_msgs::Point p)
{
    stringstream res;

    res << "[" << p.x << ", " << p.y << ", " << p.z << "]";

    return res.str();
}

std::string print(geometry_msgs::Quaternion q)
{
    stringstream res;

    res << "[" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "]";

    return res.str();
}

std::string print(geometry_msgs::Pose p)
{
    stringstream res;

    res << "[" << print(p.position) << ", " << print(p.orientation) << "]";

    return res.str();
}

void quaternionFromDoubles(geometry_msgs::Quaternion &q,
                           double x, double y, double z, double w)
{
    q.x =x;
    q.y =y;
    q.z =z;
    q.w =w;
}

/**************************************************************************/
/*                               State                                    */
/**************************************************************************/

void State::set(int _state)
{
    state = _state;

    return;
}

State::operator int()
{
    return state;
}

State::operator std::string()
{
    if      ( state == WORKING     ) { return      "WORKING"; }
    else if ( state == ERROR       ) { return        "ERROR"; }
    else if ( state == START       ) { return        "START"; }
    else if ( state == DONE        ) { return         "DONE"; }
    else if ( state == KILLED      ) { return       "KILLED"; }
    else if ( state == RECOVER     ) { return      "RECOVER"; }
    else if ( state == STOPPED     ) { return      "STOPPED"; }
    else if ( state == CTRL_RUNNING) { return "CTRL_RUNNING"; }
    else if ( state == CTRL_DONE   ) { return    "CTRL_DONE"; }
    else if ( state == CTRL_FAIL   ) { return    "CTRL_FAIL"; }
    else                             { return         "NONE"; }
}

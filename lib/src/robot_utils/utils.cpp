/**
 * Copyright (C) 2016 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 3 or any
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

string intToString( const int a )
{
    stringstream ss;
    ss << a;
    return ss.str();
}

double norm(vector<double> const& _v)
{
    double accum = 0.0;

    for (int i = 0; i < _v.size(); ++i)
    {
        accum += _v[i] * _v[i];
    }
    return sqrt(accum);
}

/**************************************************************************/
/*                               Utils                                    */
/**************************************************************************/

void State::set(int _s)
{
    state = _s;
    time  = ros::Time::now();

    return;
}

State::operator int ()
{
    return state;
}

State::operator std::string()
{
    if      ( state == WORKING  ) return "WORKING";
    else if ( state == ERROR    ) return "ERROR";
    else if ( state == START    ) return "START";
    else if ( state == DONE     ) return "DONE";
    else if ( state == KILLED   ) return "KILLED";
    else if ( state == RECOVER  ) return "RECOVER";
    else if ( state == STOPPED  ) return "STOPPED";
    else                          return "NONE";
}

State::operator ros::Time()
{
    return time;
}

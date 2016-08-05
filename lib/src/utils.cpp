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

#include "robot_interface/utils.h"

using namespace std;
using namespace geometry_msgs;

/**************************************************************************/
/*                               Utils                                    */
/**************************************************************************/

bool withinXHundredth(float x, float y, float z)
{
    float diff = abs(x - y);
    float diffTwoDP = roundf(diff * 100) / 100;
    return diffTwoDP <= (0.01 * z) ? true : false;
}

bool withinThres(float x, float y, float t)
{
    return abs(x-y)<t?true:false;
}

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

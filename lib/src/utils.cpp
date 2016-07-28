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

bool hasCollided(float range, float max_range, float min_range, string mode)
{
    float threshold;
    if(mode == "strict") threshold = 0.050;
    if(mode == "loose") threshold = 0.067;
    if(range <= max_range && range >= min_range && range <= threshold) return true;
    else return false;
}

bool hasPoseCompleted(Pose a, Pose b, string mode)
{
    // cout << "Current pose: " << a << endl;
    // cout << "Desired pose: " << b << endl;

    bool result = true;

    if(mode == "strict")
    {
        if(!equalXDP(a.position.x, b.position.x, 3)) {result = false;} 
        if(!equalXDP(a.position.y, b.position.y, 3)) {result = false;} 
    }
    else if(mode == "loose")
    {
        if(!equalXDP(a.position.x, b.position.x, 2)) {result = false;} 
        if(!equalXDP(a.position.y, b.position.y, 2)) {result = false;} 
    }

    if(!withinXHundredth(a.position.z, b.position.z, 1))       {result = false;}    
    if(!withinXHundredth(a.orientation.x, b.orientation.x, 2)) {result = false;}  
    if(!withinXHundredth(a.orientation.y, b.orientation.y, 2)) {result = false;}  
    if(!withinXHundredth(a.orientation.z, b.orientation.z, 2)) {result = false;}  
    if(!withinXHundredth(a.orientation.w, b.orientation.w, 2)) {result = false;}

    return result; 
}

bool withinXHundredth(float x, float y, float z)
{
    float diff = abs(x - y);
    float diffTwoDP = roundf(diff * 100) / 100;
    return diffTwoDP <= (0.01 * z) ? true : false;
}

bool equalXDP(float x, float y, float z)
{
    float xTwoDP = roundf(x * pow(10, z)) / pow(10, z);
    float yTwoDP = roundf(y * pow(10, z)) / pow(10, z);
    return xTwoDP == yTwoDP ? true : false;    
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

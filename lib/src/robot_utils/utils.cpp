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

/**************************************************************************/
/*                               Utils                                    */
/**************************************************************************/

State::operator int ()
{
    return state;
}

State::operator std::string()
{
    if      ( state == WORKING  ) return "WORKING";
    else if ( state == ERROR    ) return "ERROR";
    else if ( state == START    ) return "START";
    else if ( state == REST     ) return "REST";
    else if ( state == SCANNED  ) return "SCANNED";
    else if ( state == PICK_UP  ) return "PICK_UP";
    else if ( state == PUT_DOWN ) return "PUT_DOWN";
    else if ( state == DONE     ) return "DONE";
    else if ( state == KILLED   ) return "KILLED";
    else                          return "NONE";
}

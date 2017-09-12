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

#ifndef __PERCEPTION_CLIENT_IMPL__
#define __PERCEPTION_CLIENT_IMPL__

#include "robot_perception/client_template.h"

#include <aruco_msgs/MarkerArray.h>

class PerceptionClientImpl : public ClientTemplate<int>
{
protected:
    /**
     * Callback function for the ARuco topic
     *
     * @param _msg the topic message
     */
    void ObjectCb(const aruco_msgs::MarkerArray& _msg)
    {
        ROS_INFO_COND(ct_print_level>=4, "ObjectCb");

        if (_msg.markers.size() > 0)
        {
            available_objects.clear();
        }

        for (size_t i = 0; i < _msg.markers.size(); ++i)
        {
            // ROS_DEBUG("Processing object with id %i",_msg.markers[i].id);

            available_objects.push_back(int(_msg.markers[i].id));
            objects_found = true;

            if (int(_msg.markers[i].id) == getObjectID())
            {
                curr_object_pos = _msg.markers[i].pose.pose.position;
                curr_object_ori = _msg.markers[i].pose.pose.orientation;

                ROS_DEBUG("Object is in: %g %g %g", curr_object_pos.x,
                                                    curr_object_pos.y,
                                                    curr_object_pos.z);
                // ROS_INFO("Object is in: %g %g %g %g", curr_object_ori.x,
                //                                       curr_object_ori.y,
                //                                       curr_object_ori.z,
                //                                       curr_object_ori.w);

                if (!object_found) { object_found = true; }
            }
        }

        if (not is_ok) { is_ok = true; }
    };

public:
    /**
     * Constructor
     */
    PerceptionClientImpl(std::string _name, std::string _limb) :
                         ClientTemplate(_name, _limb)
    {
        sub = ctnh.subscribe("/markers/"+getClientLimb(), SUBSCRIBER_BUFFER,
                                     &PerceptionClientImpl::ObjectCb, this);

        object_id = -1;
    };

    /**
     * Destructor
     */
    ~PerceptionClientImpl() { };

};

#endif

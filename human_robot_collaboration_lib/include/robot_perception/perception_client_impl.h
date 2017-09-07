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

class ARucoClient : public ClientTemplate<int>
{
protected:
    /**
     * Callback function for the ARuco topic
     *
     * @param _msg the topic message
     */
    void ObjectCb(const aruco_msgs::MarkerArray& _msg);

public:
    /**
     * Constructor
     */
    ARucoClient(std::string _name, std::string _limb);

    /**
     * Destructor
     */
    ~ARucoClient();

};

#include <human_robot_collaboration_msgs/ObjectsArray.h>

class CartesianEstimatorClient : public ClientTemplate<std::string>
{
protected:
    /**
     * Callback function for the CartesianEstimator topic
     *
     * @param _msg the topic message
     */
    void ObjectCb(const human_robot_collaboration_msgs::ObjectsArray& _msg);

public:
    /**
     * Constructor
     */
    CartesianEstimatorClient(std::string _name, std::string _limb);

    /**
     * Destructor
     */
    ~CartesianEstimatorClient();
};

#endif

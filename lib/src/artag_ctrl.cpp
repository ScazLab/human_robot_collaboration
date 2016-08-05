#include "robot_interface/artag_ctrl.h"

#include <iostream>

#include <tf/transform_datatypes.h>

using namespace std;
using namespace baxter_collaboration;

ARTagCtrl::ARTagCtrl(std::string _name, std::string _limb) : 
                     ArmCtrl(_name,_limb), aruco_ok(false), marker_found(false)
{
    _aruco_sub = _n.subscribe("/aruco_marker_publisher/markers",
                               SUBSCRIBER_BUFFER, &ARTagCtrl::ARucoCb, this);

    elap_time = 0;

    if (!goHome()) setState(ERROR);
}

// Protected
bool ARTagCtrl::doAction(int s, std::string a)
{
    clearMarkerPose();

    if (a == ACTION_GET && (s == START ||
                            s == ERROR ||
                            s == DONE  ))
    {
        if (pickObject())
        {
            setState(PICK_UP);
            return true;
        }
        else recoverFromError();
    }
    else if (a == ACTION_PASS && s == PICK_UP)
    {
        if(passObject())
        {
            setState(DONE);
            return true;
        }
        else recoverFromError();
    }
    else if (a == ACTION_HAND_OVER && (s == START ||
                                       s == ERROR ||
                                       s == DONE  ))
    {
        if (handOver())
        {
            setState(DONE);
            return true;
        }
        else recoverFromError();
    }
    else
    {
        ROS_ERROR("[%s] Invalid State %i", getLimb().c_str(), s);
    }

    return false;
}

bool ARTagCtrl::handOver()
{
    if (!pickObject())              return false;
    if (!prepare4HandOver())        return false;
    if (!waitForOtherArm())         return false;
    ros::Duration(1.0).sleep();
    if (!releaseObject())           return false;
    if (!goHome())                  return false;

    return true;
}

bool ARTagCtrl::pickObject()
{
    if (!hoverAbovePool())          return false;
    ros::Duration(0.1).sleep();
    if (!pickARTag())               return false;
    if (!hoverAbovePool())          return false;
    if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool ARTagCtrl::passObject()
{
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!releaseObject())               return false;
    if (!goHome())                      return false;

    return true;
}

bool ARTagCtrl::prepare4HandOver()
{
    if (!hoverAboveTable(Z_LOW,"loose",true))                              return false;
    if (!goToPose(0.65, 0.085, Z_LOW-0.01, VERTICAL_ORI_L, "loose", true)) return false;  

    return true;  
}

bool ARTagCtrl::waitForOtherArm(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    string other_limb = getLimb() == "right" ? "left" : "right";

    ROS_INFO("[%s] Waiting for %s arm", getLimb().c_str(), other_limb.c_str());
 
    ros::ServiceClient _c;
    string service_name = "/"+getName()+"/service_"+other_limb+"_to_"+getLimb();
    _c = _n.serviceClient<AskFeedback>(service_name);

    AskFeedback srv;
    srv.request.ask = "ready";

    while(ros::ok())
    {
        if (disable_coll_av)      suppressCollisionAv();
        if (!_c.call(srv)) break;

        if (srv.response.reply == "gripped")
        {
            return true;
        }

        ROS_DEBUG("[%s] Received: %s ", getLimb().c_str(), srv.response.reply.c_str());

        ros::spinOnce();
        ros::Rate(100).sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No feedback from other arm has been received in %gs!",_wait_time);
            return false;
        }        
    }

    return false;
}

void ARTagCtrl::ARucoCb(const aruco_msgs::MarkerArray& msg) 
{
    for (int i = 0; i < msg.markers.size(); ++i)
    {
        // ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        if (msg.markers[i].id == getMarkerID())
        {
            _curr_marker_pos = msg.markers[i].pose.pose.position;
            _curr_marker_ori = msg.markers[i].pose.pose.orientation;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_marker_pos.x,
                                                _curr_marker_pos.y,
                                                _curr_marker_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", _curr_marker_ori.x,
            //                                       _curr_marker_ori.y,
            //                                       _curr_marker_ori.z,
            //                                       _curr_marker_ori.w);

            if (!marker_found)
            {
                marker_found = true;
            }
        }
    }

    if (!aruco_ok)
    {
        aruco_ok = true;
    }
}

bool ARTagCtrl::pickARTag()
{
    ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());

    if (!is_ir_ok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        return false;
    }

    if (!waitForARucoData()) return false;

    geometry_msgs::Quaternion q;
    if (getAction() == ACTION_HAND_OVER)
    {
        // If we have to hand_over, let's pre-orient the end effector such that
        // further movements are easier
        q = computeHOorientation();

        ROS_DEBUG("Going to: %g %g %g", _curr_marker_pos.x, _curr_marker_pos.y, getPos().z);
        if (!goToPose(_curr_marker_pos.x, _curr_marker_pos.y, getPos().z,
                                                            q.x,q.y,q.z,q.w,"loose"))
        {
            return false;
        }
    }
    
    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int ik_failures      =                0;

    while(ros::ok())
    {
        ros::Time now_time = ros::Time::now();
        double new_elap_time = (now_time - start_time).toSec();

        double x = _curr_marker_pos.x;
        double y = _curr_marker_pos.y;
        double z = z_start - PICK_UP_SPEED * new_elap_time;

        ROS_DEBUG("Time %g Going to: %g %g %g", new_elap_time, x, y, z);

        bool res=false;

        if (getAction() == ACTION_HAND_OVER)
        {
            // q   = computeHOorientation();
            res = goToPoseNoCheck(x,y,z,q.x,q.y,q.z,q.w);
        }
        else
        {
            res = goToPoseNoCheck(x,y,z,VERTICAL_ORI_L);
        }

        if (res == true)
        {
            ik_failures = 0;
            if (new_elap_time - elap_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            }
            elap_time = new_elap_time;

            if(hasCollided("strict")) 
            {
                ROS_DEBUG("Collision!");
                break;
            }

            ros::spinOnce();
            ros::Rate(100).sleep();
        }
        else
        {
            ik_failures++;
        }

        if (ik_failures == 20)
        {
            return false;
        }
    }
    
    ROS_INFO("Picking up tag..");
    return gripObject();
}

geometry_msgs::Quaternion ARTagCtrl::computeHOorientation()
{
    // Get the rotation matrix for the marker, as retrieved from ARuco
    tf::Quaternion mrk_q;
    tf::quaternionMsgToTF(_curr_marker_ori, mrk_q);
    tf::Matrix3x3 mrk_rot(mrk_q);

    // printf("Marker Orientation\n");
    // for (int j = 0; j < 3; ++j)
    // {
    //     printf("%g\t%g\t%g\n", mrk_rot[j][0], mrk_rot[j][1], mrk_rot[j][2]);
    // }

    // Compute the transform matrix between the marker's orientation 
    // and the end-effector's orientation
    tf::Matrix3x3 mrk2ee;
    mrk2ee[0][0] =  1;  mrk2ee[0][1] =  0;   mrk2ee[0][2] =  0;
    mrk2ee[1][0] =  0;  mrk2ee[1][1] =  0;   mrk2ee[1][2] = -1;
    mrk2ee[2][0] =  0;  mrk2ee[2][1] =  1;   mrk2ee[2][2] =  0;

    // printf("Rotation\n");
    // for (int j = 0; j < 3; ++j)
    // {
    //     printf("%g\t%g\t%g\n", mrk2ee[j][0], mrk2ee[j][1], mrk2ee[j][2]);
    // }

    // Compute the final end-effector orientation, and convert it to a msg
    mrk2ee = mrk_rot * mrk2ee;
    
    tf::Quaternion ee_q;
    mrk2ee.getRotation(ee_q);
    geometry_msgs::Quaternion ee_q_msg;
    tf::quaternionTFToMsg(ee_q,ee_q_msg);

    // printf("Desired Orientation\n");
    // for (int j = 0; j < 3; ++j)
    // {
    //     printf("%g\t%g\t%g\n", mrk2ee[j][0], mrk2ee[j][1], mrk2ee[j][2]);
    // }

    return ee_q_msg;
}

bool ARTagCtrl::waitForARucoData()
{
    printf("I-m here\n");
    int cnt=0;
    while (!aruco_ok)
    {
        ROS_WARN("No callback from ARuco. Is ARuco running?");
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("No callback from ARuco! Stopping.");
            return false;
        }

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    cnt=0;
    while (!marker_found)
    {
        ROS_WARN("Object with ID %i not found. Is the object there?", getMarkerID());
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("Object with ID %i not found! Stopping.", getMarkerID());
            return false;
        }

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    return true;
}

void ARTagCtrl::clearMarkerPose()
{
    aruco_ok     = false;
    marker_found = false;
}

bool ARTagCtrl::hoverAbovePool()
{
    return goToPose(POOL_POS_L, POOL_ORI_L);
}

bool ARTagCtrl::moveObjectTowardHuman()
{
    return goToPose(0.80, 0.26, 0.32, HORIZONTAL_ORI_L);
}

ARTagCtrl::~ARTagCtrl()
{

}

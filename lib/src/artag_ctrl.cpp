#include "robot_interface/artag_ctrl.h"

#include <iostream>

#include <tf/transform_datatypes.h>

using namespace std;

ARTagCtrl::ARTagCtrl(std::string _name, std::string _limb) : 
                                      ArmCtrl(_name,_limb)
{
    _aruco_sub = _n.subscribe("/aruco_marker_publisher/markers",
                               SUBSCRIBER_BUFFER, &ARTagCtrl::ArucoCb, this);

    elapsed_time = 0;

    _curr_marker_pos.x = 100;

    if (!goHome()) setState(ERROR);
}

// Protected
void ARTagCtrl::InternalThreadEntry()
{
    clearMarkerPose();

    int    s = int(getState());
    string a =     getAction();

    setState(WORKING);

    if (a == ACTION_HOME)
    {
        if (goHome())   setState(START);
        else            setState(ERROR);
    }
    else if (a == ACTION_RELEASE)
    {
        if (releaseObject())   setState(START);
        else                   setState(ERROR);
    }
    else if (a == ACTION_GET && (s == START ||
                                 s == ERROR ||
                                 s == DONE  ))
    {
        if (pickObject())   setState(PICK_UP);
        else                recoverFromError();
    }
    else if (a == ACTION_PASS && s == PICK_UP)
    {
        if(passObject())   setState(DONE);
        else               recoverFromError();
    }
    else if (a == ACTION_HAND_OVER && (s == START ||
                                       s == ERROR ||
                                       s == DONE  ))
    {
        if (handOver())    setState(DONE);
        else               recoverFromError();
    }
    else
    {
        ROS_ERROR("Invalid State %i", s);
        setState(ERROR);
    }

    pthread_exit(NULL);
    return;
}

bool ARTagCtrl::handOver()
{
    if (!hoverAboveTable(POS_HIGH)) return false;
    if (!pickARTag())               return false;
    if (!hoverAboveTable(POS_LOW))  return false;
    if (!passObject())              return false;
    // if (!releaseObject())           return false;
    // if (!goHome())                  return false;

    return true;
}

bool ARTagCtrl::pickObject()
{
    if (!hoverAbovePool())          return false;
    ros::Duration(0.1).sleep();
    if (!pickARTag())               return false;
    if (!hoverAbovePool())          return false;
    if (!hoverAboveTable(POS_LOW))  return false;

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

void ARTagCtrl::ArucoCb(const aruco_msgs::MarkerArray& msg) 
{
    for (int i = 0; i < msg.markers.size(); ++i)
    {
        ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        // if (msg.markers[i].id == getMarkerID())
        {
            i=0;
            _curr_marker_pos = msg.markers[i].pose.pose.position;
            _curr_marker_ori = msg.markers[i].pose.pose.orientation;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_marker_pos.x,
                                                _curr_marker_pos.y,
                                                _curr_marker_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", _curr_marker_ori.x,
            //                                       _curr_marker_ori.y,
            //                                       _curr_marker_ori.z,
            //                                       _curr_marker_ori.w);

            tf::Quaternion _marker_quat;
            tf::quaternionMsgToTF(_curr_marker_ori, _marker_quat);
            tf::Matrix3x3 _marker_mat(_marker_quat);

            printf("Marker Orientation\n");
            for (int j = 0; j < 3; ++j)
            {
                printf("%g\t%g\t%g\n", _marker_mat[j][0], _marker_mat[j][1], _marker_mat[j][2]);
            }
            printf("\n");
        }
    }
}

bool ARTagCtrl::pickARTag()
{
    ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());
    ros::Time start_time = ros::Time::now();

    if (!is_ir_ok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        return false;
    }

    int cnt=0;
    while (_curr_marker_pos.x == 100)
    {
        ROS_WARN("No callback from ARuco, or object with ID %i not found.", getMarkerID());
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("No object with ID %i found. Stopping.", getMarkerID());
            return false;
        }

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    double z_start  = getPos().z;

    int ik_failures = 0;
    while(ros::ok())
    {
        ros::Time now_time = ros::Time::now();
        double new_elapsed_time = (now_time - start_time).toSec();

        double x = _curr_marker_pos.x;
        double y = _curr_marker_pos.y;
        double z = z_start - PICK_UP_SPEED * new_elapsed_time;

        ROS_DEBUG("Time %g Going to: %g %g %g", new_elapsed_time, x, y, z);

        if (ARTagCtrl::goToPose(x,y,z,VERTICAL_ORIENTATION_LEFT_ARM) == true)
        {
            ik_failures = 0;
            if (new_elapsed_time - elapsed_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\t\tTime elapsed: %g", new_elapsed_time - elapsed_time);
            }
            elapsed_time = new_elapsed_time;

            if(hasCollided("strict")) 
            {
                ROS_INFO("Collision!");
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
    
    return gripObject();
}

bool ARTagCtrl::goToPose(double px, double py, double pz,
                               double ox, double oy, double oz, double ow)
{
    geometry_msgs::PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.header.stamp    = ros::Time::now();

    setPosition(   req_pose_stamped.pose, px, py, pz);
    setOrientation(req_pose_stamped.pose, ox, oy, oz, ow);

    vector<double> joint_angles;
    if (!getJointAngles(req_pose_stamped,joint_angles)) return false;

    baxter_core_msgs::JointCommand joint_cmd;
    joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    joint_cmd.command.resize(7);
    setJointNames(joint_cmd);

    for(int i = 0; i < 7; i++) {
        joint_cmd.command[i] = joint_angles[i];
    }

    publish_joint_cmd(joint_cmd);

    return true;
}

void ARTagCtrl::clearMarkerPose()
{
    _curr_marker_pos.x = 100;
}

bool ARTagCtrl::hoverAbovePool()
{
    return ROSThread::goToPose(POOL_POSITION_LEFT_ARM, VERTICAL_ORIENTATION_LEFT_ARM);
}

bool ARTagCtrl::moveObjectTowardHuman()
{
    return ROSThread::goToPose(0.80, 0.26, 0.32, HORIZONTAL_ORIENTATION_LEFT_ARM);
}

ARTagCtrl::~ARTagCtrl()
{

}

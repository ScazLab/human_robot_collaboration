#include "robot_interface/artag_controller.h"

using namespace std;

ARTagController::ARTagController(std::string limb) : ROSThread(limb), Gripper(limb),
                                                     marker_id(-1), action("")
{
    setState(START);

    _aruco_sub = _n.subscribe("/aruco_marker_publisher/markers",
                               SUBSCRIBER_BUFFER, &ARTagController::ARCallback, this);

    elapsed_time = 0;

    _curr_marker_pose.position.x = 100;
}

ARTagController::~ARTagController() { }

// Protected
void ARTagController::InternalThreadEntry()
{
    clearMarkerPose();

    int s = int(getState());

    setState(WORKING);

    if (action == ACTION_HOME)
    {
        if (goHome())   setState(START);
        else            setState(ERROR);
    }
    else if (action == ACTION_RELEASE)
    {
        if (releaseObject())   setState(START);
        else                   setState(ERROR);
    }
    else if (action == ACTION_GET && (s == START ||
                                      s == ERROR ||
                                      s == PASSED))
    {
        if (pickObject())   setState(PICK_UP);
        else                recoverFromError();
    }
    else if (action == ACTION_PASS && s == PICK_UP)
    {
        if(passObject())   setState(PASSED);
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

bool ARTagController::pickObject()
{
    if (!hoverAbovePool())          return false;
    ros::Duration(0.5).sleep();
    if (!pickARTag())               return false;
    if (!hoverAboveTable(POS_LOW))  return false;

    return true;
}

bool ARTagController::passObject()
{
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!releaseObject())               return false;
    if (!goHome())                      return false;

    return true;
}

void ARTagController::ARCallback(const aruco_msgs::MarkerArray& msg) 
{
    for (int i = 0; i < msg.markers.size(); ++i)
    {
        ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        if (msg.markers[i].id == marker_id)
        {
            _curr_marker_pose = msg.markers[i].pose.pose;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_marker_pose.position.x,
                                                _curr_marker_pose.position.y,
                                                _curr_marker_pose.position.z);
        }
    }
}

bool ARTagController::pickARTag()
{
    ROS_INFO("Start Picking up tag..");
    ros::Time start_time = ros::Time::now();

    if (!is_ir_ok())
    {
        ROS_ERROR("I didn't receive a callback from the IR sensor! Stopping.");
        return false;
    }

    int cnt=0;
    while (_curr_marker_pose.position.x == 100)
    {
        ROS_WARN("No callback from ARuco, or object with ID %i not found.", marker_id);
        ++cnt;

        if (cnt == 10)
        {
            ROS_ERROR("No object with ID %i found. Stopping.", marker_id);
            return false;
        }

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    int ik_failures = 0;
    double z_start  = _curr_pose.position.z;
    while(ros::ok())
    {
        ros::Time now_time = ros::Time::now();
        double new_elapsed_time = (now_time - start_time).toSec();

        double x = _curr_marker_pose.position.x;
        double y = _curr_marker_pose.position.y;
        double z = z_start - PICK_UP_SPEED * new_elapsed_time;

        ROS_DEBUG("Time %g Going to: %g %g %g", new_elapsed_time, x, y, z);

        if (ARTagController::goToPose(x,y,z,VERTICAL_ORIENTATION_LEFT_ARM) == true)
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

bool ARTagController::goToPose(double px, double py, double pz,
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

    publish(joint_cmd);

    return true;
}


void ARTagController::recoverFromError()
{
    releaseObject();
    goHome();
    setState(ERROR);
}

bool ARTagController::goHome()
{
    return hoverAboveTable(POS_LOW);
}

void ARTagController::clearMarkerPose()
{
    _curr_marker_pose.position.x = 100;
}

bool ARTagController::hoverAbovePool()
{
    return ROSThread::goToPose(POOL_POSITION_LEFT_ARM, VERTICAL_ORIENTATION_LEFT_ARM);
}

bool ARTagController::hoverAboveTable(double height)
{
    return ROSThread::goToPose(HOME_POSITION_LEFT_ARM, height, VERTICAL_ORIENTATION_LEFT_ARM);
}

bool ARTagController::moveObjectTowardHuman()
{
    return ROSThread::goToPose(0.80, 0.26, 0.32, HORIZONTAL_ORIENTATION_LEFT_ARM);
}


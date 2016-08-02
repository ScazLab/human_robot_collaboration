#include "robot_interface/ros_thread.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace cv;

/*
    BoardStateSensing error at start
    Drop token faster
    Flag option for turning off display windows when run as part of baxterTictactoe
    Get rid of imageScreen node error and show something else via boardScheme
    Error-checking when cellsDefinitionAuto does not see board
*/

/**************************************************************************/
/*                            ROSThread                                   */
/**************************************************************************/
ROSThread::ROSThread(string limb): _limb(limb), _state(START,0), spinner(4)
{
    _joint_cmd_pub = _n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);   
    _endpt_sub     = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state", SUBSCRIBER_BUFFER, &ROSThread::endpointCallback, this);
    _ir_sub        = _n.subscribe("/robot/range/" + _limb + "_hand_range/state", SUBSCRIBER_BUFFER, &ROSThread::IRCallback, this);
    _ik_client     = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb + "/PositionKinematicsNode/IKService");

    _init_time = ros::Time::now();

    _curr_max_range = 0;
    _curr_min_range = 0;
    _curr_range     = 0;

    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);

    spinner.start();
}

ROSThread::~ROSThread() { }

bool ROSThread::startInternalThread()
{
    return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
}

void ROSThread::WaitForInternalThreadToExit() {(void) pthread_join(_thread, NULL);}

void ROSThread::endpointCallback(const baxter_core_msgs::EndpointState& msg) 
{
    ROS_DEBUG("endpointCallback");
    _curr_pose     = msg.pose;
    _curr_position = _curr_pose.position;
    _curr_wrench   = msg.wrench;

    filterForces();
}

void ROSThread::IRCallback(const sensor_msgs::RangeConstPtr& msg) 
{
    ROS_DEBUG("IRCallback");
    _curr_range = msg->range; 
    _curr_max_range = msg->max_range; 
    _curr_min_range = msg->min_range;
}

void ROSThread::filterForces()
{
    _filt_force[0] = (1 - FORCE_ALPHA) * _filt_force[0] + FORCE_ALPHA * _curr_wrench.force.x;
    _filt_force[1] = (1 - FORCE_ALPHA) * _filt_force[1] + FORCE_ALPHA * _curr_wrench.force.y;
    _filt_force[2] = (1 - FORCE_ALPHA) * _filt_force[2] + FORCE_ALPHA * _curr_wrench.force.z;
}

void ROSThread::hoverAboveTokens(double height)
{
    goToPose(0.540, 0.570, height, VERTICAL_ORIENTATION_LEFT_ARM);
}

bool ROSThread::goToPose(double px, double py, double pz,
                         double ox, double oy, double oz, double ow,
                         std::string mode)
{
    PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.header.stamp    = ros::Time::now();

    setPosition(   req_pose_stamped.pose, px, py, pz);
    setOrientation(req_pose_stamped.pose, ox, oy, oz, ow);

    vector<double> joint_angles;
    if (!getJointAngles(req_pose_stamped,joint_angles)) return false;

    while(ros::ok)
    {
        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;

        // joint_cmd.names
        setJointNames(joint_cmd);
        joint_cmd.command.resize(7);
        // joint_cmd.angles
        for(int i = 0; i < joint_angles.size(); i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        // ROS_INFO("Publishing joint commands.. %g",
        //           ros::Time::now().toSec()-_init_time.toSec());
        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(100).sleep();
        ros::spinOnce();

        if(hasPoseCompleted(_curr_pose, req_pose_stamped.pose, mode)) 
        {
            break;
        }
    }

    return true;
}

bool ROSThread::getJointAngles(geometry_msgs::PoseStamped& pose_stamped,
                                      std::vector<double>& joint_angles)
{
    joint_angles.clear();
    bool got_solution = false;
    ros::Time start = ros::Time::now();
    float thresh_z = pose_stamped.pose.position.z + 0.120;

    while(!got_solution)
    {
        SolvePositionIK ik_srv;
        //ik_srv.request.seed_mode=2;         // i.e. SEED_CURRENT
        ik_srv.request.seed_mode=0;         // i.e. SEED_AUTO
        pose_stamped.header.stamp=ros::Time::now();
        ik_srv.request.pose_stamp.push_back(pose_stamped);
        
        if(_ik_client.call(ik_srv))
        {
            got_solution = ik_srv.response.isValid[0];

            if (got_solution)
            {
                joint_angles = ik_srv.response.joints[0].position;
                return true;
            }
            else
            {
                // if position cannot be reached, try a position with the same x-y coordinates
                // but higher z (useful when placing tokens)
                ROS_WARN("IK solution not valid: %g %g %g", pose_stamped.pose.position.x,
                                                            pose_stamped.pose.position.y,
                                                            pose_stamped.pose.position.z);
                pose_stamped.pose.position.z += 0.004;
            } 
        }

        // if no solution is found within 200 milliseconds or no solution within the acceptable
        // z-coordinate threshold is found, then no solution exists and exit oufof loop
        if((ros::Time::now() - start).toSec() > 0.2 || pose_stamped.pose.position.z > thresh_z) 
        {
            ROS_ERROR("Did not find a suitable IK solution! Part %s Final Position %g %g %g",
                                                                        getLimb().c_str(),
                                                             pose_stamped.pose.position.x,
                                                             pose_stamped.pose.position.y,
                                                             pose_stamped.pose.position.z);
            return false;
        }
    }

    return false;
}

void ROSThread::setJointNames(JointCommand& joint_cmd)
{
    joint_cmd.names.push_back(_limb + "_s0");
    joint_cmd.names.push_back(_limb + "_s1");
    joint_cmd.names.push_back(_limb + "_e0");
    joint_cmd.names.push_back(_limb + "_e1");
    joint_cmd.names.push_back(_limb + "_w0");
    joint_cmd.names.push_back(_limb + "_w1");
    joint_cmd.names.push_back(_limb + "_w2");
}

bool ROSThread::detectForceInteraction()
{
    double f_x = abs(_curr_wrench.force.x - _filt_force[0]);
    double f_y = abs(_curr_wrench.force.y - _filt_force[1]);
    double f_z = abs(_curr_wrench.force.z - _filt_force[2]);

    ROS_DEBUG("Interaction: %g %g %g", f_x, f_y, f_z);

    if (f_x > FORCE_THRES || f_y > FORCE_THRES || f_z > FORCE_THRES)
    {
        ROS_INFO("Interaction: %g %g %g", f_x, f_y, f_z);
        return true;
    }
    else
    {
        return false;
    }
}

bool ROSThread::waitForForceInteraction(double _wait_time)
{
    ros::Time _init = ros::Time::now();

    while(ros::ok)
    {
        if (detectForceInteraction()) return true;

        ros::Rate(100).sleep();
        ros::spinOnce();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No force interaction has been detected in %gs!",_wait_time);
            return false;
        }
    }
}

void ROSThread::setState(int state)
{
    _state.state = state;
    // store the time elapsed between object initialization and state change
    _state.time = (ros::Time::now() - _init_time).toSec();
}

// for syncing mutex locks (crash/errors occur if not used)
// pause() changes timing of execution of thread locks, but unclear
// why crash occurs w/o it and needs to be investigated
void ROSThread::pause()
{
    ros::Duration(0.001).sleep();
}

// Private
void * ROSThread::InternalThreadEntryFunc(void * This) 
{
    ((ROSThread *)This)->InternalThreadEntry(); 
    return NULL;
}

/**************************************************************************/
/*                          ROSThreadImage                                */
/**************************************************************************/

ROSThreadImage::ROSThreadImage(string limb): _img_trp(_n), ROSThread(limb)
{
    _img_sub = _img_trp.subscribe("/cameras/left_hand_camera/image", SUBSCRIBER_BUFFER, &ROSThreadImage::imageCallback, this);
    pthread_mutex_init(&_mutex_img, NULL);
}

ROSThreadImage::~ROSThreadImage() {}

void ROSThreadImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("imageCallback");
    cv_bridge::CvImageConstPtr cv_ptr;
    try {cv_ptr = cv_bridge::toCvShare(msg);}
    catch(cv_bridge::Exception& e) { ROS_ERROR("[Arm Controller] cv_bridge exception: %s", e.what()); }
 
    pthread_mutex_lock(&_mutex_img);
    _curr_img = cv_ptr->image.clone();
    _curr_img_size = _curr_img.size();
    _curr_img_empty = _curr_img.empty();
    pthread_mutex_unlock(&_mutex_img);   
}


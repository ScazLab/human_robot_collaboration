#include "robot_interface/robot_interface.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace cv;

/**************************************************************************/
/*                         RobotInterface                                 */
/**************************************************************************/
RobotInterface::RobotInterface(string name, string limb, bool no_robot, bool use_forces, bool use_trac_ik) :
                               _n(name), _name(name), _limb(limb), _state(START), spinner(4), ir_ok(false),
                               _no_robot(no_robot), is_ctrl_running(false), ik_solver(limb, no_robot),
                               _use_forces(use_forces), _use_trac_ik(use_trac_ik)
{
    if (no_robot) return;

    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);

    pthread_mutex_init(&_mutex_jnts, &_mutex_attr);
    pthread_mutex_init(&_mutex_ctrl, &_mutex_attr);

    _joint_cmd_pub = _n.advertise<JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);
    _coll_av_pub   = _n.advertise<Empty>("/robot/limb/" + _limb + "/suppress_collision_avoidance", 1);

    _endpt_sub     = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::endpointCb, this);

    _ir_sub        = _n.subscribe("/robot/range/" + _limb + "_hand_range/state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::IRCb, this);
    _cuff_sub      = _n.subscribe("/robot/digital_io/" + _limb + "_lower_button/state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::cuffCb, this);

    _jntstate_sub  = _n.subscribe("/robot/joint_states",
                                    SUBSCRIBER_BUFFER, &RobotInterface::jointStatesCb, this);

    _coll_av_sub   = _n.subscribe("/robot/limb/" + _limb + "/collision_avoidance_state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::collAvCb, this);

    _ctrl_sub      = _n.subscribe("/" + _name + "/limb/" + _limb + "/go_to_pose",
                                    SUBSCRIBER_BUFFER, &RobotInterface::ctrlMsgCb, this);

    if (!_use_trac_ik)
    {
        _ik_client = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb +
                                                       "/PositionKinematicsNode/IKService");
    }

    _curr_max_range = 0;
    _curr_min_range = 0;
    _curr_range     = 0;

    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);

    if (getLimb()=="left")
    {
        _n.param<double>("force_threshold_left",  force_thres, FORCE_THRES_L);
    }
    else if (getLimb()=="right")
    {
        _n.param<double>("force_threshold_right", force_thres, FORCE_THRES_R);
    }
    ROS_INFO("[%s] Force Threshold : %g", getLimb().c_str(), force_thres);

    spinner.start();
    startThread();
}

bool RobotInterface::startThread()
{
    return _thread.start(ThreadEntryFunc);
}

bool RobotInterface::closeThread()
{
    return _thread.close();
}

bool RobotInterface::killThread()
{
    return _thread.kill();
}

void RobotInterface::ThreadEntry()
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);

    ros::Rate r(THREAD_FREQ);

    while (RobotInterface::ok())
    {
        // ROS_INFO("Time: %g", (ros::Time::now() - initTime).toSec());

        if (isCtrlRunning())
        {
            double time_elap = (ros::Time::now() - time_start).toSec();

            geometry_msgs::Point      p_d = pose_des.position;
            geometry_msgs::Quaternion o_d = pose_des.orientation;

            if (!isPoseReached(p_d.x, p_d.y, p_d.z,
                               o_d.x, o_d.y, o_d.z, o_d.w))
            {
                /* code */
            }
            else
            {
                setCtrlRunning(false);
            }

        }
        r.sleep();
    }

    closeThread();
    return;
}

void * RobotInterface::ThreadEntryFunc(void * obj)
{
    ((RobotInterface *)obj)->ThreadEntry();
    return NULL;
}

bool RobotInterface::ok()
{
    bool res = ros::ok();
    res = res && getState() != KILLED && getState() != STOPPED;

    return res;
}

bool RobotInterface::getIKLimits(KDL::JntArray &ll, KDL::JntArray &ul)
{
    return ik_solver.getKDLLimits(ll,ul);
}

bool RobotInterface::setIKLimits(KDL::JntArray ll, KDL::JntArray ul)
{
    ik_solver.setKDLLimits(ll,ul);
    return true;
}

bool RobotInterface::initCtrlParams()
{
    time_start = ros::Time::now();
    pose_start = getPose();
}

void RobotInterface::ctrlMsgCb(const baxter_collaboration::GoToPose& msg)
{
    pose_des.position = msg.pose_stamp.pose.position;

    if (msg.pose_stamp.pose.orientation.x != -100 &&
        msg.pose_stamp.pose.orientation.y != -100 &&
        msg.pose_stamp.pose.orientation.z != -100 &&
        msg.pose_stamp.pose.orientation.w != -100)
    {
        pose_des.orientation = msg.pose_stamp.pose.orientation;
    }
    else
    {
        pose_des.orientation = getOri();
    }

    ctrl_mode = msg.ctrl_mode;

    if (ctrl_mode != baxter_collaboration::GoToPose::POSITION_MODE )
    {
        ROS_WARN("As of now, the only accepted control mode is POSITION_MODE");
        ctrl_mode = baxter_collaboration::GoToPose::POSITION_MODE;
    }

    setCtrlRunning(true);
    initCtrlParams();

    return;
}

void RobotInterface::setCtrlRunning(bool _flag)
{
    pthread_mutex_lock(&_mutex_ctrl);
    is_ctrl_running = _flag;
    pthread_mutex_unlock(&_mutex_ctrl);

    return;
}

bool RobotInterface::isCtrlRunning()
{
    bool res;

    pthread_mutex_lock(&_mutex_ctrl);
    res = is_ctrl_running;
    pthread_mutex_unlock(&_mutex_ctrl);

    return res;
}

void RobotInterface::collAvCb(const baxter_core_msgs::CollisionAvoidanceState& msg)
{
    if (msg.collision_object.size()!=0)
    {
        is_colliding =  true;

        string objects = "";
        for (int i = 0; i < msg.collision_object.size(); ++i)
        {
            objects = objects + " " + msg.collision_object[i];
        }
        ROS_WARN("[%s] Collision detected with: %s", getLimb().c_str(), objects.c_str());
    }
    else is_colliding = false;

    return;
}

void RobotInterface::jointStatesCb(const sensor_msgs::JointState& msg)
{
    JointCommand joint_cmd;
    setJointNames(joint_cmd);

    if (msg.name.size() >= joint_cmd.names.size())
    {
        pthread_mutex_lock(&_mutex_jnts);
        _curr_jnts.name.clear();
        _curr_jnts.position.clear();
        for (int i = 0; i < joint_cmd.names.size(); ++i)
        {
            for (int j = 0; j < msg.name.size(); ++j)
            {
                if (joint_cmd.names[i] == msg.name[j])
                {
                    _curr_jnts.name.push_back(msg.name[j]);
                    _curr_jnts.position.push_back(msg.position[j]);
                }
            }
        }
        pthread_mutex_unlock(&_mutex_jnts);
    }

    return;
}

void RobotInterface::cuffCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        setState(KILLED);
    }

    return;
}

void RobotInterface::endpointCb(const baxter_core_msgs::EndpointState& msg)
{
    ROS_DEBUG("endpointCb");
    _curr_pos      = msg.pose.position;
    _curr_ori      = msg.pose.orientation;

    if (_use_forces == true) _curr_wrench   = msg.wrench;

    tf::Quaternion _marker_quat;
    tf::quaternionMsgToTF(_curr_ori, _marker_quat);
    tf::Matrix3x3 _marker_mat(_marker_quat);

    // printf("Endpoint Orientation\n");
    // for (int j = 0; j < 3; ++j)
    // {
    //     printf("%g\t%g\t%g\n", _marker_mat[j][0], _marker_mat[j][1], _marker_mat[j][2]);
    // }
    // printf("\n");

    if (_use_forces == true) filterForces();

    return;
}

void RobotInterface::IRCb(const sensor_msgs::RangeConstPtr& msg)
{
    ROS_DEBUG("IRCb");
    _curr_range = msg->range;
    _curr_max_range = msg->max_range;
    _curr_min_range = msg->min_range;

    if (!ir_ok)
    {
        ir_ok = true;
    }

    return;
}

void RobotInterface::filterForces()
{
    _filt_force[0] = (1 - FORCE_ALPHA) * _filt_force[0] + FORCE_ALPHA * _curr_wrench.force.x;
    _filt_force[1] = (1 - FORCE_ALPHA) * _filt_force[1] + FORCE_ALPHA * _curr_wrench.force.y;
    _filt_force[2] = (1 - FORCE_ALPHA) * _filt_force[2] + FORCE_ALPHA * _curr_wrench.force.z;

    return;
}

void RobotInterface::hoverAboveTokens(double height)
{
    goToPose(0.540, 0.570, height, VERTICAL_ORI_L);

    return;
}

bool RobotInterface::goToPoseNoCheck(double px, double py, double pz,
                                     double ox, double oy, double oz, double ow)
{
    vector<double> joint_angles;
    if (!computeIK(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    return goToPoseNoCheck(joint_angles);
}

bool RobotInterface::goToPoseNoCheck(vector<double> joint_angles)
{
    JointCommand joint_cmd;
    joint_cmd.mode = JointCommand::POSITION_MODE;

    setJointNames(joint_cmd);

    for (int i = 0; i < joint_angles.size(); i++)
    {
        joint_cmd.command.push_back(joint_angles[i]);
    }

    publish_joint_cmd(joint_cmd);

    return true;
}

bool RobotInterface::goToPose(double px, double py, double pz,
                         double ox, double oy, double oz, double ow,
                         std::string mode, bool disable_coll_av)
{
    vector<double> joint_angles;
    if (!computeIK(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    ros::Rate r(100);
    while (RobotInterface::ok())
    {
        if (disable_coll_av)
        {
            suppressCollisionAv();
        }
        else
        {
            if (is_colliding == true)
            {
                ROS_ERROR("Collision Occurred! Stopping.");
                return false;
            }
        }

        if (!goToPoseNoCheck(joint_angles))   return false;

        if (isPoseReached(px, py, pz, ox, oy, oz, ow, mode))
        {
            return true;
        }

        r.sleep();
    }

    return false;
}

bool RobotInterface::computeIK(double px, double py, double pz,
                                   double ox, double oy, double oz, double ow,
                                   std::vector<double>& joint_angles)
{
    PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "base";
    pose_stamp.header.stamp    = ros::Time::now();

    setPosition(   pose_stamp.pose, px, py, pz);
    setOrientation(pose_stamp.pose, ox, oy, oz, ow);

    joint_angles.clear();
    bool got_solution = false;
    ros::Time start = ros::Time::now();
    float thresh_z = pose_stamp.pose.position.z + 0.01;

    while (!got_solution)
    {
        SolvePositionIK ik_srv;

        pose_stamp.header.stamp=ros::Time::now();

        //ik_srv.request.seed_mode=2;       // i.e. SEED_CURRENT
        ik_srv.request.seed_mode=0;         // i.e. SEED_AUTO

        ik_srv.request.pose_stamp.push_back(pose_stamp);
        pthread_mutex_lock(&_mutex_jnts);
        ik_srv.request.seed_angles.push_back(_curr_jnts);
        pthread_mutex_unlock(&_mutex_jnts);

        int cnt = 0;
        ros::Time tn = ros::Time::now();

        bool result = _use_trac_ik?ik_solver.perform_ik(ik_srv):_ik_client.call(ik_srv);

        if(result)
        {
            double te  = ros::Time::now().toSec()-tn.toSec();;
            if (te>0.010)
            {
                ROS_WARN_ONCE("\t\t\tTime elapsed in computing IK: %g cnt %i",te,cnt);
            }
            cnt++;

            if (ik_srv.response.isValid[0])
            {
                ROS_DEBUG("Got solution!");
                joint_angles = ik_srv.response.joints[0].position;
                return true;
            }
            else
            {
                // if position cannot be reached, try a position with the same x-y coordinates
                // but higher z (useful when placing tokens)
                ROS_DEBUG("[%s] IK solution not valid: %g %g %g", getLimb().c_str(),
                                                         pose_stamp.pose.position.x,
                                                         pose_stamp.pose.position.y,
                                                         pose_stamp.pose.position.z);
                pose_stamp.pose.position.z += 0.001;
            }
        }

        // if no solution is found within 50 milliseconds or no solution within the acceptable
        // z-coordinate threshold is found, then no solution exists and exit out of loop
        if ((ros::Time::now() - start).toSec() > 0.05 || pose_stamp.pose.position.z > thresh_z)
        {
            ROS_WARN("[%s] Did not find a suitable IK solution! Final Position %g %g %g",
                                                                       getLimb().c_str(),
                                                              pose_stamp.pose.position.x,
                                                              pose_stamp.pose.position.y,
                                                              pose_stamp.pose.position.z);
            return false;
        }
    }

    return false;
}

bool RobotInterface::hasCollided(string mode)
{
    float thres;

    if     (mode == "strict") thres = 0.050;
    else if (mode ==  "loose") thres = 0.067;

    if (_curr_range <= _curr_max_range &&
       _curr_range >= _curr_min_range &&
       _curr_range <= thres) return true;
    else return false;
}

bool RobotInterface::isPoseReached(double px, double py, double pz,
                                   double ox, double oy, double oz, double ow, string mode)
{
    if (!isPositionReached(px, py, pz, mode))         return false;
    if (!isOrientationReached(ox, oy, oz, ow, mode))  return false;

    return true;
}

bool RobotInterface::isPositionReached(double px, double py, double pz, string mode)
{
    ROS_DEBUG("[%s] Checking %s position. Error: %g %g %g", getLimb().c_str(),
                   mode.c_str(), px-getPos().x, py-getPos().y, pz-getPos().z);

    if (mode == "strict")
    {
        if (abs(getPos().x-px) > 0.005) return false;
        if (abs(getPos().y-py) > 0.005) return false;
        if (abs(getPos().z-pz) > 0.005) return false;
    }
    else if (mode == "loose")
    {
        if (abs(getPos().x-px) > 0.01) return false;
        if (abs(getPos().y-py) > 0.01) return false;
        if (abs(getPos().z-pz) > 0.01) return false;
    }
    else
    {
        ROS_ERROR("[%s] Please specify a mode of operation!", getLimb().c_str());
        return false;
    }

    return true;
}

bool RobotInterface::isOrientationReached(double ox, double oy, double oz, double ow, string mode)
{
    tf::Quaternion des(ox,oy,oz,ow);
    tf::Quaternion cur;
    tf::quaternionMsgToTF(getOri(), cur);

    ROS_DEBUG("[%s] Checking    orientation. Current %g %g %g %g Desired %g %g %g %g Dot %g",
                           getLimb().c_str(), getOri().x, getOri().y, getOri().z, getOri().w,
                                                                  ox,oy,oz,ow, des.dot(cur));

    if (abs(des.dot(cur)) < 0.985)  return false;

    return true;
}

bool RobotInterface::isConfigurationReached(baxter_core_msgs::JointCommand joint_cmd, std::string mode)
{
    if (_curr_jnts.position.size() < 7)
    {
        return false;
    }

    ROS_DEBUG("[%s] Checking configuration: Current %g %g %g %g %g %g %g\tDesired %g %g %g %g %g %g %g",
                                                                                      getLimb().c_str(),
         _curr_jnts.position[0], _curr_jnts.position[1], _curr_jnts.position[2], _curr_jnts.position[3],
                                 _curr_jnts.position[4], _curr_jnts.position[5], _curr_jnts.position[6],
                 joint_cmd.command[0], joint_cmd.command[1], joint_cmd.command[2], joint_cmd.command[3],
                                       joint_cmd.command[4], joint_cmd.command[5], joint_cmd.command[6]);

    bool result = false;
    for (int i = 0; i < joint_cmd.names.size(); ++i)
    {
        bool res = false;
        for (int j = 0; j < _curr_jnts.name.size(); ++j)
        {
            if (joint_cmd.names[i] == _curr_jnts.name[j])
            {
                if (mode == "strict")
                {
                    // It's approximatively half a degree
                    if (abs(joint_cmd.command[i]-_curr_jnts.position[j]) > 0.010) return false;
                }
                else if (mode == "loose")
                {
                    // It's approximatively a degree
                    if (abs(joint_cmd.command[i]-_curr_jnts.position[j]) > 0.020) return false;
                }
                res = true;
            }
        }
        if (res == false)   return false;
    }

    return true;
}

void RobotInterface::setJointNames(JointCommand& joint_cmd)
{
    joint_cmd.names.push_back(getLimb() + "_s0");
    joint_cmd.names.push_back(getLimb() + "_s1");
    joint_cmd.names.push_back(getLimb() + "_e0");
    joint_cmd.names.push_back(getLimb() + "_e1");
    joint_cmd.names.push_back(getLimb() + "_w0");
    joint_cmd.names.push_back(getLimb() + "_w1");
    joint_cmd.names.push_back(getLimb() + "_w2");
}

void RobotInterface::setJointCommands(double s0, double s1, double e0, double e1,
                                                 double w0, double w1, double w2,
                                      baxter_core_msgs::JointCommand& joint_cmd)
{
    joint_cmd.command.push_back(s0);
    joint_cmd.command.push_back(s1);
    joint_cmd.command.push_back(e0);
    joint_cmd.command.push_back(e1);
    joint_cmd.command.push_back(w0);
    joint_cmd.command.push_back(w1);
    joint_cmd.command.push_back(w2);
}

bool RobotInterface::detectForceInteraction()
{
    double f_x = abs(_curr_wrench.force.x - _filt_force[0]);
    double f_y = abs(_curr_wrench.force.y - _filt_force[1]);
    double f_z = abs(_curr_wrench.force.z - _filt_force[2]);

    ROS_DEBUG("Interaction: %g %g %g", f_x, f_y, f_z);

    if (f_x > force_thres || f_y > force_thres || f_z > force_thres)
    {
        ROS_INFO("[%s] Interaction: %g %g %g", getLimb().c_str(), f_x, f_y, f_z);
        return true;
    }
    else
    {
        return false;
    }
}

bool RobotInterface::waitForForceInteraction(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    ros::Rate r(100);
    while (RobotInterface::ok())
    {
        if (disable_coll_av)          suppressCollisionAv();
        if (detectForceInteraction())           return true;

        r.sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No force interaction has been detected in %gs!",_wait_time);
            return false;
        }
    }

    return false;
}

geometry_msgs::Pose RobotInterface::getPose()
{
    geometry_msgs::Pose res;

    res.position    = getPos();
    res.orientation = getOri();

    return res;
}

void RobotInterface::setState(int state)
{
    _state.set(state);
}

void RobotInterface::publish_joint_cmd(baxter_core_msgs::JointCommand _cmd)
{
    _joint_cmd_pub.publish(_cmd);
}

void RobotInterface::suppressCollisionAv()
{
    std_msgs::Empty empty_cmd;
    _coll_av_pub.publish(empty_cmd);
}

RobotInterface::~RobotInterface()
{
    pthread_mutex_destroy(&_mutex_jnts);
    pthread_mutex_destroy(&_mutex_ctrl);

    _thread.kill();
}

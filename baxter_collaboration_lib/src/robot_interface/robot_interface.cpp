#include "robot_interface/robot_interface.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace baxter_core_msgs;

/**************************************************************************/
/*                         RobotInterface                                 */
/**************************************************************************/
RobotInterface::RobotInterface(string _name, string _limb, bool _use_robot, double _ctrl_freq, bool _use_forces,
                               bool _use_trac_ik, bool _use_cart_ctrl, bool _is_experimental) : nh(_name), name(_name),
                               limb(_limb), state(START), spinner(4), use_robot(_use_robot), use_forces(_use_forces),
                               ir_ok(false), curr_range(0.0), curr_min_range(0.0), curr_max_range(0.0),
                               ik_solver(_limb, _use_robot), use_trac_ik(_use_trac_ik), ctrl_freq(_ctrl_freq),
                               filt_force{0.0, 0.0, 0.0}, filt_change{0.0, 0.0, 0.0}, time_filt_last_updated(ros::Time::now()),
                               is_coll_av_on(false), is_coll_det_on(false), use_cart_ctrl(_use_cart_ctrl), is_ctrl_running(false),
                               is_experimental(_is_experimental), ctrl_mode(baxter_collaboration_msgs::GoToPose::POSITION_MODE),
                               ctrl_type("pose")
{
    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);

    pthread_mutex_init(&_mutex_jnts, &_mutex_attr);
    pthread_mutex_init(&_mutex_ctrl, &_mutex_attr);

    // if (not _use_robot) return;

    if (getLimb()=="left")
    {
        nh.param<double>("force_threshold_left",  force_thres, FORCE_THRES_L);
        nh.param<double>("force_filter_variance_left", filt_variance, FORCE_FILT_VAR_L);
        nh.param<double>("relative_force_threshold_left", rel_force_thres, REL_FORCE_THRES_L);
    }
    else if (getLimb()=="right")
    {
        nh.param<double>("force_threshold_right", force_thres, FORCE_THRES_R);
        nh.param<double>("force_filter_variance_right", filt_variance, FORCE_FILT_VAR_R);
        nh.param<double>("relative_force_threshold_right", rel_force_thres, REL_FORCE_THRES_R);
    }

    ROS_INFO("[%s] ctrlFreq set to %g [Hz]", getLimb().c_str(), getCtrlFreq());
    ROS_INFO("[%s] Force Threshold : %g", getLimb().c_str(), force_thres);
    ROS_INFO("[%s] Force Filter Variance: %g", getLimb().c_str(), filt_variance);
    ROS_INFO("[%s] Relative Force Threshold: %g", getLimb().c_str(), rel_force_thres);

    ROS_INFO("[%s] Cartesian Controller %s enabled", getLimb().c_str(), use_cart_ctrl?"is":"is NOT");

    joint_cmd_pub  = nh.advertise<JointCommand>("/robot/limb/" + getLimb() + "/joint_command", 1);
    coll_av_pub    = nh.advertise<std_msgs::Empty>("/robot/limb/" + getLimb() + "/suppress_collision_avoidance", 1);

    endpt_sub      = nh.subscribe("/robot/limb/" + getLimb() + "/endpoint_state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::endpointCb, this);

    ir_sub         = nh.subscribe("/robot/range/" + getLimb() + "_hand_range/state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::IRCb, this);

    cuff_sub_lower = nh.subscribe("/robot/digital_io/" + getLimb() + "_lower_button/state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::cuffLowerCb, this);

    cuff_sub_upper = nh.subscribe("/robot/digital_io/" + getLimb() + "_upper_button/state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::cuffUpperCb, this);

    jntstate_sub   = nh.subscribe("/robot/joint_states",
                                   SUBSCRIBER_BUFFER, &RobotInterface::jointStatesCb, this);

    coll_av_sub    = nh.subscribe("/robot/limb/" + getLimb() + "/collision_avoidance_state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::collAvCb, this);

    coll_det_sub   = nh.subscribe("/robot/limb/" + getLimb() + "/collision_detection_state",
                                   SUBSCRIBER_BUFFER, &RobotInterface::collDetCb, this);

    std::string topic = "/"+getName()+"/"+getLimb()+"/state";
    state_pub = nh.advertise<baxter_collaboration_msgs::ArmState>(topic, SUBSCRIBER_BUFFER, true);
    ROS_INFO("[%s] Created state publisher with name : %s", getLimb().c_str(), topic.c_str());

    if (use_cart_ctrl)
    {
        string topic = "/" + getName() + "/" + getLimb() + "/go_to_pose";
        ctrl_sub     = nh.subscribe(topic, SUBSCRIBER_BUFFER, &RobotInterface::ctrlMsgCb, this);
        ROS_INFO("[%s] Created cartesian controller that listens to : %s", getLimb().c_str(), topic.c_str());
    }

    if (not use_trac_ik)
    {
        ik_client = nh.serviceClient<SolvePositionIK>("/ExternalTools/" + getLimb() +
                                                       "/PositionKinematicsNode/IKService");
    }

    spinner.start();

    if (use_cart_ctrl)
    {
        startThread();
        setState(START);
    }

    if (is_experimental)    ROS_WARN("[%s] Experimental mode enabled!", getLimb().c_str());
}

bool RobotInterface::startThread()
{
    return thread.start(ThreadEntryFunc, this);
}

bool RobotInterface::closeThread()
{
    return thread.close();
}

bool RobotInterface::killThread()
{
    return thread.kill();
}

void RobotInterface::ThreadEntry()
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    ros::Rate r(ctrl_freq);

    while (RobotInterface::ok())
    {
        // ROS_INFO("Time: %g", (ros::Time::now() - initTime).toSec());

        if (isCtrlRunning())
        {
            double time_elap = (ros::Time::now() - time_start).toSec();

            // Starting pose in terms of position and orientation
            geometry_msgs::Point      p_s =    pose_start.position;
            geometry_msgs::Quaternion o_s = pose_start.orientation;

            // Desired  pose in terms of position and orientation
            geometry_msgs::Point      p_d =      pose_des.position;
            geometry_msgs::Quaternion o_d =   pose_des.orientation;

            if (!isPoseReached(p_d, o_d, "strict", getCtrlType()))
            {
                // Current pose to send to the IK solver.
                geometry_msgs::Pose pose_curr = pose_des;

                /* POSITIONAL PART */
                // We model the end effector as a 3D point that moves toward the
                // target with a straight trajectory and constant speed.
                geometry_msgs::Point p_c = p_s + (p_d - p_s) / norm(p_d - p_s) * ARM_SPEED * time_elap;

                // Check if the current position is overshooting the desired position
                // By checking the sign of the cosine of the angle between p_d-p_s and p_d-p_c
                // This would mean equal to 1 within some small epsilon (1e-8)
                if (dot(p_d-p_s, p_d-p_c)/(norm(p_d-p_s)*norm(p_d-p_c)) - 1 <  EPSILON &&
                    dot(p_d-p_s, p_d-p_c)/(norm(p_d-p_s)*norm(p_d-p_c)) - 1 > -EPSILON)
                {
                    pose_curr.position    = p_c;
                }

                /* ORIENTATIONAL PART */
                // We use a spherical linear interpolation between o_s and o_d. The speed of the interpolation
                // depends on ARM_ROT_SPEED, which is fixed and defined in utils.h
                tf::Quaternion o_d_TF, o_s_TF;
                tf::quaternionMsgToTF(o_d, o_d_TF);
                tf::quaternionMsgToTF(o_s, o_s_TF);
                double angle     = o_s_TF.angleShortestPath(o_d_TF);
                double traj_time =            angle / ARM_ROT_SPEED;

                if (time_elap < traj_time)
                {
                    tf::Quaternion o_c_TF = o_s_TF.slerp(o_d_TF, time_elap / traj_time);
                    o_c_TF.normalize();
                    geometry_msgs::Quaternion o_c;
                    tf::quaternionTFToMsg(o_c_TF, o_c);

                    pose_curr.orientation = o_c;
                }

                // ROS_INFO("[%s] Current Pose: %s Time %g/%g", getLimb().c_str(), print(pose_curr).c_str(),
                //                                                                    time_elap, traj_time);

                if (!goToPoseNoCheck(pose_curr))
                {
                    ROS_WARN("[%s] desired configuration could not be reached.", getLimb().c_str());
                    setCtrlRunning(false);
                    setState(CTRL_FAIL);
                }

                if (hasCollidedIR("strict")) ROS_INFO_THROTTLE(2, "[%s] is colliding!", getLimb().c_str());
            }
            else
            {
                ROS_INFO("[%s] Pose reached!\n", getLimb().c_str());

                if (ctrl_mode == baxter_collaboration_msgs::GoToPose::VELOCITY_MODE)
                {
                    vector<double> joint_values;
                    for (int i = 0; i < 7; ++i)
                    {
                        joint_values.push_back(0.0);
                    }
                    goToJointConfNoCheck(joint_values);
                }
                setCtrlRunning(false);
                setState(CTRL_DONE);
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
    return true;
}

void RobotInterface::ctrlMsgCb(const baxter_collaboration_msgs::GoToPose& msg)
{
    if (int(getState()) != WORKING)
    {
        // First, let's check if the type of the control command is allowed
        if (msg.type == "stop")
        {
            ROS_INFO("[%s] Stopping cartesian controller server.", getLimb().c_str());
            setCtrlRunning(false);
            return;
        }

        if  (msg.type ==   "position" || msg.type ==       "pose" ||
             msg.type == "relative_x" || msg.type == "relative_y" || msg.type == "relative_z")
        {
            if (msg.type == "pose")
            {
                pose_des.position    = msg.position;
                pose_des.orientation = msg.orientation;
            }
            else
            {
                if (msg.type == "position")
                {
                    pose_des.position = msg.position;
                }
                else
                {
                    pose_des.position = getPos();
                }

                pose_des.orientation = getOri();
            }

            if (msg.type == "relative_x")
            {
                pose_des.position.x += msg.increment;
            }
            if (msg.type == "relative_y")
            {
                pose_des.position.y += msg.increment;
            }
            if (msg.type == "relative_z")
            {
                pose_des.position.z += msg.increment;
            }
        }
        else
        {
            ROS_ERROR("[%s] Requested command type %s not allowed!",
                                  getLimb().c_str(), msg.type.c_str());
            return;
        }

        // Then, let's check if control mode is among the allowed options
        if (msg.ctrl_mode != baxter_collaboration_msgs::GoToPose::POSITION_MODE)
        {
            if (not is_experimental)
            {
                ROS_ERROR("[%s] As of now, the only tested control mode is POSITION_MODE. "
                          "To be able to use any other control mode, please set the "
                          "experimental flag in the constructor to true.", getLimb().c_str());
                return;
            }
            else
            {
                if (msg.ctrl_mode == baxter_collaboration_msgs::GoToPose::VELOCITY_MODE)
                {
                    ROS_WARN("[%s] Experimental VELOCITY_MODE enabled", getLimb().c_str());
                    ctrl_mode = baxter_collaboration_msgs::GoToPose::VELOCITY_MODE;
                }
                else if (msg.ctrl_mode == baxter_collaboration_msgs::GoToPose::RAW_POSITION_MODE)
                {
                    ROS_WARN("[%s] Experimental RAW_POSITION_MODE enabled", getLimb().c_str());
                    ctrl_mode = baxter_collaboration_msgs::GoToPose::RAW_POSITION_MODE;
                }
                else
                {
                    ROS_ERROR("[%s] Requested control mode %i not allowed!",
                                          getLimb().c_str(), msg.ctrl_mode);
                    return;
                }
            }
        }

        ctrl_mode = msg.ctrl_mode;

        setCtrlRunning(true);
        initCtrlParams();

        ROS_INFO("[%s] Received new target pose: %s mode: %i", getLimb().c_str(), print(pose_des).c_str(), ctrl_mode);
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "[%s] Received new target control command, but the controller is already"
                              " in use through the high level interface!", getLimb().c_str());
    }

    return;
}

void RobotInterface::setCtrlRunning(bool _flag)
{
    // ROS_INFO("[%s] Setting is_ctrl_running to: %i", getLimb().c_str(), _flag);

    pthread_mutex_lock(&_mutex_ctrl);
    is_ctrl_running = _flag;
    pthread_mutex_unlock(&_mutex_ctrl);

    if (_flag == true)    { setState(CTRL_RUNNING); }
    else                  { setState(   CTRL_DONE); }

    return;
}

bool RobotInterface::isCtrlRunning()
{
    bool res;

    pthread_mutex_lock(&_mutex_ctrl);
    res = is_ctrl_running;
    pthread_mutex_unlock(&_mutex_ctrl);

    // ROS_INFO("[%s] is_ctrl_running equal to: %i", "left", res);

    return res;
}

void RobotInterface::collAvCb(const baxter_core_msgs::CollisionAvoidanceState& msg)
{
    if (msg.collision_object.size()!=0)
    {
        is_coll_av_on =  true;

        string objects = "";
        for (size_t i = 0; i < msg.collision_object.size(); ++i)
        {
            // Let's remove the first part of the collision object name for visualization
            // purposes, i.e. the part that says "collision_"
            objects = objects + " " + string(msg.collision_object[i]).erase(0,10);
        }
        ROS_WARN_THROTTLE(1, "[%s] Collision avoidance with: %s",
                             getLimb().c_str(), objects.c_str());
    }
    else is_coll_av_on = false;

    return;
}

void RobotInterface::collDetCb(const baxter_core_msgs::CollisionDetectionState& msg)
{
    if (msg.collision_state==true)
    {
        is_coll_det_on = true;

        ROS_WARN_THROTTLE(1, "[%s] Collision detected!", getLimb().c_str());
    }
    else is_coll_det_on = false;

    return;
}

void RobotInterface::jointStatesCb(const sensor_msgs::JointState& msg)
{
    JointCommand joint_cmd;
    setJointNames(joint_cmd);

    if (msg.name.size() >= joint_cmd.names.size())
    {
        pthread_mutex_lock(&_mutex_jnts);

        // cout << "Joint state ";
        // for (size_t i = 9; i < 16; ++i)
        // {
        //     cout << "[" << i << "] " << msg.name[i] << " " << msg.position[i] << "\t";
        // }
        // cout << endl;
        curr_jnts.name.clear();
        curr_jnts.position.clear();
        curr_jnts.velocity.clear();

        for (size_t i = 0; i < joint_cmd.names.size(); ++i)
        {
            for (size_t j = 0; j < msg.name.size(); ++j)
            {
                if (joint_cmd.names[i] == msg.name[j])
                {
                    curr_jnts.name.push_back(msg.name[j]);
                    curr_jnts.position.push_back(msg.position[j]);
                    curr_jnts.velocity.push_back(msg.velocity[j]);
                }
            }
        }
        pthread_mutex_unlock(&_mutex_jnts);
    }

    return;
}

void RobotInterface::cuffLowerCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        setState(KILLED);
    }

    return;
}

void RobotInterface::cuffUpperCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        setState(KILLED);
    }

    return;
}

void RobotInterface::endpointCb(const baxter_core_msgs::EndpointState& msg)
{
    // ROS_DEBUG("endpointCb");
    curr_pos = msg.pose.position;
    curr_ori = msg.pose.orientation;

    if (use_forces == true)
    {
        curr_wrench = msg.wrench;
        filterForces();
    }

    return;
}

void RobotInterface::IRCb(const sensor_msgs::Range& msg)
{
    // ROS_DEBUG("IRCb");
    curr_range     = msg.range;
    curr_max_range = msg.max_range;
    curr_min_range = msg.min_range;

    if (!ir_ok)
    {
        ir_ok = true;
    }

    return;
}

void RobotInterface::filterForces()
{
    double time_elap = ros::Time::now().toSec() - time_filt_last_updated.toSec();

    vector<double> new_filt;
    vector<double> predicted_filt;

    // initial attempt to update filter using a running average of the forces on the arm (exponential moving average)
    new_filt.push_back((1 - FORCE_ALPHA) * filt_force[0] + FORCE_ALPHA * curr_wrench.force.x);
    new_filt.push_back((1 - FORCE_ALPHA) * filt_force[1] + FORCE_ALPHA * curr_wrench.force.y);
    new_filt.push_back((1 - FORCE_ALPHA) * filt_force[2] + FORCE_ALPHA * curr_wrench.force.z);

    for (int i = 0; i < 3; ++i)
    {
        // extrapolate a predicted new filter value using the previous rate of change of the filter value:
        // new value = old value + rate of change * elapsed time
        predicted_filt.push_back(filt_force[i] + (filt_change[i] * time_elap));

        // update the rate of change of the filter using the new value from the initial attempt above
        filt_change[i] = (new_filt[i] - filt_force[i])/time_elap;

        // if the predicted filter value is very small or 0, this is most likely the first time the filter is updated
        // (the filter values and rate of change start at 0), so set the filter to the new value from the initial attempt above
        if (predicted_filt[i] < FILTER_EPSILON)
        {
            filt_force[i] = new_filt[i];
        }
        else
        {
            // compare the initial attempt to the predicted filter value
            // if the relative difference is within a threshold defined in utils.h, update the filter to the new value from the initial attempt
            // otherwise, the filter is not changed; this keeps the filter from changing wildly while maintaining trends in the data
            if (abs((new_filt[i] - predicted_filt[i])/predicted_filt[i]) < filt_variance)
            {
                filt_force[i] = new_filt[i];
            }
        }
    }

    time_filt_last_updated = ros::Time::now();

}

bool RobotInterface::goToPoseNoCheck(geometry_msgs::Pose p)
{
    return goToPoseNoCheck(p.position, p.orientation);
}

bool RobotInterface::goToPoseNoCheck(geometry_msgs::Point p, geometry_msgs::Quaternion o)
{
    return goToPoseNoCheck(p.x, p.y, p.z, o.x, o.y, o.z, o.w);
}

bool RobotInterface::goToPoseNoCheck(double px, double py, double pz,
                                     double ox, double oy, double oz, double ow)
{
    vector<double> joint_angles;
    if (!computeIK(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    return goToJointConfNoCheck(joint_angles);
}

bool RobotInterface::goToJointConfNoCheck(vector<double> joint_values)
{
    JointCommand     joint_cmd;
    joint_cmd.mode = ctrl_mode;

    setJointNames(joint_cmd);

    for (size_t i = 0; i < joint_values.size(); i++)
    {
        joint_cmd.command.push_back(joint_values[i]);
    }

    publishJointCmd(joint_cmd);

    return true;
}

bool RobotInterface::goToPose(double px, double py, double pz,
                              double ox, double oy, double oz, double ow,
                              string mode, bool disable_coll_av)
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
            if (is_coll_av_on == true)
            {
                ROS_ERROR("Collision Occurred! Stopping.");
                return false;
            }
        }

        if (!goToJointConfNoCheck(joint_angles))   return false;

        if (isPoseReached(px, py, pz, ox, oy, oz, ow, mode))
        {
            return true;
        }

        r.sleep();
    }

    return false;
}

bool RobotInterface::computeIK(geometry_msgs::Pose p, vector<double>& j)
{
    return computeIK(p.position, p.orientation, j);
}

bool RobotInterface::computeIK(geometry_msgs::Point p, geometry_msgs::Quaternion o,
                               vector<double>& j)
{
    return computeIK(p.x, p.y, p.z, o.x, o.y, o.z, o.w, j);
}

bool RobotInterface::computeIK(double px, double py, double pz,
                               double ox, double oy, double oz, double ow,
                               vector<double>& j)
{
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "base";
    pose_stamp.header.stamp    = ros::Time::now();

    setPosition(   pose_stamp.pose, px, py, pz);
    setOrientation(pose_stamp.pose, ox, oy, oz, ow);

    j.clear();
    ros::Time start = ros::Time::now();
    float thresh_z = pose_stamp.pose.position.z + 0.01;

    while (RobotInterface::ok())
    {
        SolvePositionIK ik_srv;

        pose_stamp.header.stamp=ros::Time::now();

        //ik_srv.request.seed_mode=2;       // i.e. SEED_CURRENT
        ik_srv.request.seed_mode=0;         // i.e. SEED_AUTO

        ik_srv.request.pose_stamp.push_back(pose_stamp);
        pthread_mutex_lock(&_mutex_jnts);
        ik_srv.request.seed_angles.push_back(curr_jnts);
        pthread_mutex_unlock(&_mutex_jnts);

        ros::Time tn = ros::Time::now();

        bool result = use_trac_ik?ik_solver.perform_ik(ik_srv):ik_client.call(ik_srv);

        if(result)
        {
            double te  = ros::Time::now().toSec()-tn.toSec();;
            if (te>0.010)
            {
                ROS_WARN_ONCE("\t\t\tTime elapsed in computing IK: %g",te);
            }

            if (ik_srv.response.isValid[0])
            {
                ROS_DEBUG("Got solution!");
                j = ik_srv.response.joints[0].position;
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

bool RobotInterface::hasCollidedIR(string mode)
{
    double thres = 0.0;

    if (getLimb() == "left")
    {
        if      (mode == "strict") thres = 0.050;
        else if (mode ==  "loose") thres = 0.067;
    }
    else if (getLimb() == "right")
    {
        if      (mode == "strict") thres = 0.089;
        else if (mode ==  "loose") thres = 0.110;
    }
    else
    {
        return false;
    }

    if (curr_range <= curr_max_range &&
        curr_range >= curr_min_range &&
        curr_range <= thres             ) return true;

    return false;
}

bool RobotInterface::hasCollidedCD()
{
    return is_coll_det_on;
}

bool RobotInterface::isPoseReached(geometry_msgs::Pose p, string mode, string type)
{
    return isPoseReached(p.position, p.orientation, mode, type);
}

bool RobotInterface::isPoseReached(geometry_msgs::Point p, geometry_msgs::Quaternion o,
                                   string mode, string type)
{
    return isPoseReached(p.x, p.y, p.z, o.x, o.y, o.z, o.w, mode, type);
}

bool RobotInterface::isPoseReached(double px, double py, double pz,
                                   double ox, double oy, double oz, double ow,
                                   string mode, string type)
{
    if (type == "pose" || type == "position")
    {
        if (!   isPositionReached(px, py, pz,     mode))  return false;
    }

    if (type == "pose" || type == "orientation")
    {
        if (!isOrientationReached(ox, oy, oz, ow, mode))  return false;
    }

    if (type != "pose" && type != "position" && type != "orientation")
    {
        ROS_ERROR("[%s] Type should be either pose, position or orientation."
                  " Received %s instead.", getLimb().c_str(), type.c_str());

        return false;
    }

    return true;
}

bool RobotInterface::isPositionReached(geometry_msgs::Point p, string mode)
{
    return isPositionReached(p.x, p.y, p.z, mode);
}

bool RobotInterface::isPositionReached(double px, double py, double pz, string mode)
{
    ROS_DEBUG("[%s] Checking %s position. Error: %g %g %g", getLimb().c_str(),
                   mode.c_str(), px-getPos().x, py-getPos().y, pz-getPos().z);

    if (mode == "strict")
    {
        if (abs(getPos().x-px) > 0.003) { return false; }
        if (abs(getPos().y-py) > 0.003) { return false; }
        if (abs(getPos().z-pz) > 0.003) { return false; }
    }
    else if (mode == "loose")
    {
        if (abs(getPos().x-px) > 0.010) { return false; }
        if (abs(getPos().y-py) > 0.010) { return false; }
        if (abs(getPos().z-pz) > 0.010) { return false; }
    }
    else
    {
        ROS_ERROR("[%s] Mode should be either strict or loose. Received %s instead.",
                                                    getLimb().c_str(), mode.c_str());
        return false;
    }

    return true;
}

bool RobotInterface::isOrientationReached(geometry_msgs::Quaternion q, string mode)
{
    return isOrientationReached(q.x, q.y, q.z, q.w, mode);
}

bool RobotInterface::isOrientationReached(double ox, double oy, double oz, double ow, string mode)
{
    tf::Quaternion des(ox, oy, oz, ow);
    tf::Quaternion cur;
    tf::quaternionMsgToTF(getOri(), cur);

    ROS_DEBUG("[%s] Checking %s orientation. Curr %g %g %g %g Des %g %g %g %g Dot %g",
                                       getLimb().c_str(), mode.c_str(),
                                       getOri().x, getOri().y, getOri().z, getOri().w,
                                       ox, oy, oz, ow, des.dot(cur));

    if (mode == "strict")
    {
        if (abs(des.dot(cur)) < 0.98)  { return false; }
    }
    else if (mode == "loose")
    {
        if (abs(des.dot(cur)) < 0.95)  { return false; }
    }
    else
    {
        ROS_ERROR("[%s] Mode should be either strict or loose. Received %s instead.",
                                                    getLimb().c_str(), mode.c_str());
        return false;
    }

    return true;
}

bool RobotInterface::isConfigurationReached(vector<double> des_jnts, string mode)
{
    if (curr_jnts.position.size() < 7 || des_jnts.size() < 7)
    {
        return false;
    }

    baxter_core_msgs::JointCommand dj;
    setJointNames(dj);
    setJointCommands(des_jnts[0], des_jnts[1], des_jnts[2],
                     des_jnts[3], des_jnts[4], des_jnts[5], des_jnts[6], dj);

    return isConfigurationReached(dj, mode);
}

bool RobotInterface::isConfigurationReached(baxter_core_msgs::JointCommand des_jnts, string mode)
{
    if (curr_jnts.position.size() < 7)
    {
        return false;
    }

    ROS_DEBUG("[%s] Checking configuration: Current %g %g %g %g %g %g %g\tDesired %g %g %g %g %g %g %g",
                                                                                      getLimb().c_str(),
             curr_jnts.position[0], curr_jnts.position[1], curr_jnts.position[2], curr_jnts.position[3],
                                    curr_jnts.position[4], curr_jnts.position[5], curr_jnts.position[6],
               des_jnts.command[0],   des_jnts.command[1],   des_jnts.command[2],   des_jnts.command[3],
                                      des_jnts.command[4],   des_jnts.command[5],   des_jnts.command[6]);

    for (size_t i = 0; i < des_jnts.names.size(); ++i)
    {
        bool res = false;
        for (size_t j = 0; j < curr_jnts.name.size(); ++j)
        {
            if (des_jnts.names[i] == curr_jnts.name[j])
            {
                if (mode == "strict")
                {
                    // It's approximatively half a degree
                    if (abs(des_jnts.command[i]-curr_jnts.position[j]) > 0.010) return false;
                }
                else if (mode == "loose")
                {
                    // It's approximatively a degree
                    if (abs(des_jnts.command[i]-curr_jnts.position[j]) > 0.020) return false;
                }
                res = true;
            }
        }
        if (res == false)   return false;
    }

    return true;
}

bool RobotInterface::setCtrlType(const std::string &_ctrl_type)
{
    if (_ctrl_type != "pose" && _ctrl_type != "position" && _ctrl_type != "orientation")
    {
        ROS_ERROR("[%s] Type should be either pose, position or orientation."
                  " Received %s instead.", getLimb().c_str(), _ctrl_type.c_str());

        return false;
    }

    ctrl_type = _ctrl_type;
    ROS_DEBUG("[%s] Control type set to %s", getLimb().c_str(), ctrl_type.c_str());

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

double RobotInterface::relativeDiff(double a, double b)
{
    // returns the relative difference of a to b
    // 0.01 is added to b in case it is very small (this will not affect large values of b)
    return abs((a - b)/abs(b + 0.01));
}

bool RobotInterface::detectForceInteraction()
{
    // ROS_INFO("Filt Forces: %g, %g, %g", filt_force[0], filt_force[1], filt_force[2]);

    // compare the current force to the filter force. if the relative difference is above a
    // threshold defined in utils.h, return true

    if (relativeDiff(curr_wrench.force.x, filt_force[0]) > rel_force_thres ||
        relativeDiff(curr_wrench.force.y, filt_force[1]) > rel_force_thres ||
        relativeDiff(curr_wrench.force.z, filt_force[2]) > rel_force_thres)
    {
        ROS_INFO("Interaction: %g %g %g", curr_wrench.force.x, curr_wrench.force.y, curr_wrench.force.z);
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
            ROS_WARN("No force interaction has been detected in %gs!",_wait_time);
            return false;
        }
    }

    return false;
}

bool RobotInterface::waitForJointAngles(double _wait_time)
{
    ros::Time _init = ros::Time::now();

    ros::Rate r(100);
    while (RobotInterface::ok())
    {
        sensor_msgs::JointState _jnt_state = getJointStates();
        if (_jnt_state.position.size() > 0)      return true;

        r.sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_WARN("No joint angles received in %gs!",_wait_time);
            return false;
        }
    }

    return false;
}

sensor_msgs::JointState RobotInterface::getJointStates()
{
    sensor_msgs::JointState cj;

    pthread_mutex_lock(&_mutex_jnts);
    cj = curr_jnts;
    pthread_mutex_unlock(&_mutex_jnts);

    return   cj;
}

geometry_msgs::Pose RobotInterface::getPose()
{
    geometry_msgs::Pose res;

    res.position    = getPos();
    res.orientation = getOri();

    return res;
}

bool RobotInterface::setState(int _state)
{
    state.set(_state);

    // disable the cartesian controller server
    if (state == WORKING)
    {
        setCtrlRunning(false);
    }

    return publishState();
}

bool RobotInterface::publishState()
{
    baxter_collaboration_msgs::ArmState msg;

    msg.state  = string(getState());

    state_pub.publish(msg);

    return true;
}

void RobotInterface::publishJointCmd(baxter_core_msgs::JointCommand _cmd)
{
    // cout << "Joint Command: " << _cmd << endl;
    joint_cmd_pub.publish(_cmd);
}

void RobotInterface::suppressCollisionAv()
{
    coll_av_pub.publish(std_msgs::Empty());
}

RobotInterface::~RobotInterface()
{
    pthread_mutex_destroy(&_mutex_jnts);
    pthread_mutex_destroy(&_mutex_ctrl);

    thread.kill();
}


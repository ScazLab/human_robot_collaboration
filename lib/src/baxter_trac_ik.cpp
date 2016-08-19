#include "robot_interface/baxter_trac_ik.h"

baxterTracIK::baxterTracIK(std::string limb) : _limb(limb), _urdf_param("/robot_description"),
                                               _timeout(0.05), _eps(1e-6), _num_steps(1)
{
    _tracik_solver = new TRAC_IK::TRAC_IK("base", limb + "_gripper", _urdf_param, _timeout, _eps);

    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    if(!(_tracik_solver->getKDLChain(_chain)))
    {
        ROS_ERROR("There was no valid KDL chain found");
        exit(EXIT_FAILURE);
    }

    if(!(_tracik_solver->getKDLLimits(ll,ul)))
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        exit(EXIT_FAILURE);
    }

    if(!(_chain.getNrOfJoints() == ll.data.size()) ||
       !(_chain.getNrOfJoints() == ul.data.size()))
    {
        ROS_ERROR("Inconsistent joint limits found");
        exit(EXIT_FAILURE);
    }

    // Create Nominal chain configuration midway between all joint limits
    _nominal = new KDL::JntArray(_chain.getNrOfJoints());

    for (uint j=0; j < _nominal->data.size(); j++)
    {
      _nominal->operator()(j) = (ll(j)+ul(j))/2.0;
    }
}

baxterTracIK::~baxterTracIK()
{
    delete _tracik_solver;
    delete _nominal;
}

KDL::JntArray baxterTracIK::JointState2JntArray(const sensor_msgs::JointState &js)
{
    KDL::JntArray array(_chain.getNrOfJoints());
    for(uint joint=0; joint<js.position.size(); ++joint)
    {
        array(joint) = js.position[joint];
    }
    return array;
}

bool baxterTracIK::perform_ik(baxter_core_msgs::SolvePositionIK::Request &req,
                              baxter_core_msgs::SolvePositionIK::Response &res)
{
    ROS_INFO("Beginning");
    int rc;
    KDL::JntArray result;
    sensor_msgs::JointState joint_state;
    for(uint segment=0; segment<_chain.getNrOfSegments(); ++segment)
    {
        KDL::Joint joint = _chain.getSegment(segment).getJoint();
        if(joint.getType()!=KDL::Joint::None)
        {
            joint_state.name.push_back(joint.getName());
        }
    }
    bool seeds_provided = req.seed_angles.size() == req.pose_stamp.size();

    for(uint point=0; point<req.pose_stamp.size(); ++point)
    {
        joint_state.position.clear();
        KDL::Frame end_effector_pose(KDL::Rotation::Quaternion(req.pose_stamp[point].pose.orientation.x,
                                                               req.pose_stamp[point].pose.orientation.y,
                                                               req.pose_stamp[point].pose.orientation.z,
                                                               req.pose_stamp[point].pose.orientation.w),
                                     KDL::Vector(req.pose_stamp[point].pose.position.x,
                                                 req.pose_stamp[point].pose.position.y,
                                                 req.pose_stamp[point].pose.position.z));

        KDL::JntArray seed(_chain.getNrOfJoints());
        if(seeds_provided)   seed = JointState2JntArray(req.seed_angles[point]);

        for(uint num_attempts=0; num_attempts<_num_steps; ++num_attempts)
        {
            ROS_INFO("Attempt num %i with tolerance %g", num_attempts, _eps);

            _tracik_solver->setEpsilon(_eps);
            rc = _tracik_solver->CartToJnt(seeds_provided? seed: *(_nominal), end_effector_pose, result);
            ROS_INFO("rc: %i",rc);
            if(rc>=0) break;
        }

        for(uint joint=0; joint<_chain.getNrOfJoints(); ++joint)
        {
            joint_state.position.push_back(result(joint));
        }

        res.joints.push_back(joint_state);
        res.isValid.push_back(rc>=0);
    }
    ROS_INFO("End");
    return true;
}

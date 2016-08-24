#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <sensor_msgs/JointState.h>

#include "utils.h"

class baxterTracIK
{
private:
    std::string       _limb;
    std::string _urdf_param;

    double _timeout;
    double     _eps;
    int  _num_steps;

    TRAC_IK::TRAC_IK *_tracik_solver;
    KDL::Chain _chain;
    KDL::JntArray *_nominal;

public:
    baxterTracIK(std::string limb, bool no_robot = false);

    ~baxterTracIK();

    KDL::JntArray JointState2JntArray(const sensor_msgs::JointState &js);

    bool perform_ik(IK_call &ik);

    void computeFwdKin(KDL::JntArray jointpositions);
};


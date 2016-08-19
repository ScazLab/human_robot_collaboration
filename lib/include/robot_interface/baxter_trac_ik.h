#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>

class baxterTracIK
{
private:
    std::string _limb;
    std::string _urdf_param;

    double _timeout;
    double     _eps;
    int  _num_steps;

    TRAC_IK::TRAC_IK *_tracik_solver;
    KDL::Chain _chain;
    KDL::JntArray *_nominal;

public:
    baxterTracIK(std::string limb);

    ~baxterTracIK();

    KDL::JntArray JointState2JntArray(const sensor_msgs::JointState &js);

    bool perform_ik(baxter_core_msgs::SolvePositionIK::Request &req,
                    baxter_core_msgs::SolvePositionIK::Response &res);

    void computeFwdKin(KDL::JntArray jointpositions);
};


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "trac_ik_baxter");
//   ros::NodeHandle nh;

//   std::string urdf_param;
//   double timeout;


//   nh.param("timeout", timeout, 0.005);
//   nh.param("urdf_param", urdf_param, std::string("/robot_description"));

//   baxterTracIK left("left", timeout, urdf_param);
//   ros::ServiceServer left_service = nh.advertiseService("trac_ik_left", &baxterTracIK::perform_ik, &left);

//   baxterTracIK right("right", timeout, urdf_param);
//   ros::ServiceServer right_service = nh.advertiseService("trac_ik_right", &baxterTracIK::perform_ik, &right);

//   ros::spin();

//   return 0;
// }

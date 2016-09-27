#include "tower_building/cube_picker.h"

using namespace std;
using namespace baxter_core_msgs;

CubePicker::CubePicker(std::string _name, std::string _limb, bool _no_robot) :
                       ARTagCtrl(_name, _limb, _no_robot)
{

}

bool CubePicker::hoverAboveTableStrict(bool disable_coll_av)
{
    ROS_INFO("[%s] Hovering above table strict..", getLimb().c_str());

    ros::Rate r(100);
    while(ros::ok())
    {
        if (disable_coll_av)    suppressCollisionAv();

        JointCommand joint_cmd;
        joint_cmd.mode = JointCommand::POSITION_MODE;
        setJointNames(joint_cmd);

        joint_cmd.command.push_back( 0.19673303604630432);  //'left_s0'
        joint_cmd.command.push_back(-0.870150601928001);    //'left_s1'
        joint_cmd.command.push_back(-1.0530778108833365);   //'left_e0'
        joint_cmd.command.push_back( 1.5577574900976376);   //'left_e1'
        joint_cmd.command.push_back( 0.6515583396543295);   //'left_w0'
        joint_cmd.command.push_back( 1.2463593901568986);   //'left_w1'
        joint_cmd.command.push_back(-0.1787087617886507);   //'left_w2'

        publish_joint_cmd(joint_cmd);

        r.sleep();

        if(isPoseReached(HOME_POS_L, Z_LOW, VERTICAL_ORI_L))
        {
            ROS_INFO("[%s] Done", getLimb().c_str());
            return true;
        }
    }
    return false;
}

CubePicker::~CubePicker()
{

}

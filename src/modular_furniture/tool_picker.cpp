#include "tool_picker.h"

using namespace std;
using namespace baxter_core_msgs;

ToolPicker::ToolPicker(std::string _name, std::string _limb, bool _no_robot) :
                       ArmCtrl(_name,_limb, _no_robot), CartesianEstimatorClient(_name, _limb),
                       elap_time(0)
{
    setHomeConfiguration();

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));

    printActionDB();

    insertObject(1, "screwdriver");

    printObjectDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ToolPicker::pickUpObject()
{
    ROS_INFO("[%s] Start Picking up object %s..", getLimb().c_str(),
              CartesianEstimatorClient::getObjectName().c_str());

    if (!is_ir_ok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        return false;
    }

    if (!waitForCartEstData()) return false;

    geometry_msgs::Quaternion q;

    double x = getObjectPos().x+0.02;
    double y = getObjectPos().y-0.02;
    double z =       getPos().z;

    ROS_INFO("Going to: %g %g %g", x, y, z);
    if (!goToPose(x, y, z, POOL_ORI_R,"loose"))
    {
        return false;
    }

    if (!waitForCartEstData()) return false;

    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int cnt_ik_fail      =                0;

    double period = 100.0;
    ros::Rate r(period);

    while(RobotInterface::ok())
    {
        double new_elap_time = (ros::Time::now() - start_time).toSec();

        // geometry_msgs::Point p_n = getPos();
        // geometry_msgs::Point p_d = getObjectPos();
        // p_d.x += 0.02;
        // p_d.y -= 0.02;
        // geometry_msgs::Point p_c = p_n + (p_d - p_n) / norm(p_d - p_n) * ARM_SPEED * (1/period);

        x = getObjectPos().x + 0.02;
        y = getObjectPos().y - 0.02;
        z = z_start - ARM_SPEED * new_elap_time / 1.3;

        ROS_DEBUG("Time %g Going to: %g %g %g Position: %g %g %g", new_elap_time, x, y, z,
                                                       getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,POOL_ORI_R))
        {
            cnt_ik_fail = 0;
            if (new_elap_time - elap_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            }
            elap_time = new_elap_time;

            if(hasCollided("strict"))
            {
                ROS_INFO("Collision!");
                setSubState(ACTION_GET);
                return true;
            }

            r.sleep();
        }
        else
        {
            cnt_ik_fail++;
        }

        if (cnt_ik_fail == 10)
        {
            return false;
        }
    }

    return false;
}

bool ToolPicker::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();
    if (!pickUpObject())            return false;
    if (!gripObject())              return false;
    // if (!moveArm("up", 0.3))        return false;
    // if (!hoverAboveTable(Z_LOW))    return false;
    ros::Duration(0.8).sleep();
    if (!releaseObject())               return false;
    if (!homePoseStrict())              return false;

    return true;
}

bool ToolPicker::passObject()
{
    if (getSubState() != ACTION_GET)    return false;
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!releaseObject())               return false;
    if (!homePoseStrict())              return false;

    return true;
}

bool ToolPicker::getPassObject()
{
    if (!getObject())      return false;
    if (!passObject())      return false;

    return true;
}

bool ToolPicker::moveObjectTowardHuman()
{
    ROS_INFO("[%s] Moving object toward human..", getLimb().c_str());
    return goToPose(0.80, -0.26, 0.32, VERTICAL_ORI_L);
}

void ToolPicker::setHomeConfiguration()
{
    setHomeConf(-1.6801, -1.0500, 1.1693, 1.9762,
                         -0.5722, 1.0205, 0.5430);
}

void ToolPicker::setObjectID(int _obj)
{
    ArmCtrl::setObjectID(_obj);
    CartesianEstimatorClient::setObjectName(ArmCtrl::getObjectName(_obj));
}

ToolPicker::~ToolPicker()
{

}

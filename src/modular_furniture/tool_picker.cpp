#include "tool_picker.h"

using namespace std;
using namespace baxter_core_msgs;

ToolPicker::ToolPicker(std::string _name, std::string _limb, bool _no_robot) :
                       HoldCtrl(_name,_limb, _no_robot), CartesianEstimatorClient(_name, _limb),
                       elap_time(0)
{
    setHomeConfiguration();

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));
    insertAction(ACTION_CLEANUP,   static_cast<f_action>(&ToolPicker::cleanUpObject));

    printActionDB();

    if (_no_robot) return;

    reduceSquish();

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ToolPicker::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();

    if (getObjectIDs().size() >  1)
    {
        setSubState(CHECK_OBJ_IDS);
        int id = chooseObjectID(getObjectIDs());
        if (id == -1)       return false;
        setObjectID(id);
        ROS_INFO("[%s] Chosen object with ID %i", getLimb().c_str(),
                                                     getObjectID());
    }

    if (!pickUpObject())            return false;
    if (!gripObject())              return false;
    if (!moveArm("up", 0.3))        return false;
    // if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool ToolPicker::passObject()
{
    if (getPrevAction() != ACTION_GET)  return false;

    if (CartesianEstimatorClient::getObjectName() == "screwdriver")
    {
        if (!goToPose(0.85, -0.26, 0.27, HORIZONTAL_ORI_R)) return false;
        ros::Duration(1.0).sleep();
        if (!waitForForceInteraction())                     return false;
        if (!releaseObject())                               return false;
    }
    else if (CartesianEstimatorClient::getObjectName() == "screws_box"  ||
             CartesianEstimatorClient::getObjectName() == "brackets_box")
    {
        if (!hoverAboveTable(Z_LOW))    return false;

        if (CartesianEstimatorClient::getObjectName() == "brackets_box")
        {
            if (!goToPose(0.63, -0.10, -0.12, VERTICAL_ORI_R)) return false;
        }
        else
        {
            if (!goToPose(0.63, -0.30, -0.12, VERTICAL_ORI_R)) return false;
        }

        ros::Duration(0.5).sleep();
        if (!releaseObject())           return false;
        if (!hoverAboveTable(Z_LOW))    return false;
    }

    if (!homePoseStrict())              return false;

    return true;
}

bool ToolPicker::getPassObject()
{
    if (!getObject())      return false;
    setPrevAction(ACTION_GET);
    if (!passObject())     return false;

    return true;
}

bool ToolPicker::cleanUpObject()
{
    if (!goToPose(0.65, -0.25, 0.25, VERTICAL_ORI_R)) return false;
    ros::Duration(0.05).sleep();

    if (getObjectIDs().size() >  1)
    {
        setSubState(CHECK_OBJ_IDS);
        int id = chooseObjectID(getObjectIDs());
        if (id == -1)       return false;
        setObjectID(id);
        ROS_INFO("[%s] Chosen object with ID %i", getLimb().c_str(),
                                                     getObjectID());
    }

    if (!waitForCartEstObjFound())
    {
        setSubState(NO_OBJ);
        return false;
    }

    if (!pickUpObject())            return false;
    if (!gripObject())              return false;
    if (!moveArm("up", 0.3))        return false;
    if (!homePoseStrict())          return false;

    if (CartesianEstimatorClient::getObjectName() == "screwdriver")
    {
        if (!goToPose(0.20, -0.85, -0.27, POOL_ORI_R)) return false;
    }
    else if (CartesianEstimatorClient::getObjectName() == "brackets_box")
    {
        if (!goToPose(0.00, -0.85, -0.25, POOL_ORI_R)) return false;
    }
    else if (CartesianEstimatorClient::getObjectName() == "screws_box")
    {
        if (!goToPose(-0.15, -0.85, -0.25, POOL_ORI_R)) return false;
    }

    ros::Duration(0.5).sleep();
    if (!releaseObject())           return false;
    if (!homePoseStrict())          return false;

    return true;
}

bool ToolPicker::pickUpObject()
{
    ROS_INFO("[%s] Start Picking up object %s..", getLimb().c_str(),
                  CartesianEstimatorClient::getObjectName().c_str());

    if (!is_ir_ok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        setSubState(NO_IR_SENSOR);
        return false;
    }

    if (!waitForCartEstData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    double offs_x = 0.0;
    double offs_y = 0.0;

    if (!computeOffsets(offs_x, offs_y))    return false;

    double x = getObjectPos().x + offs_x;
    double y = getObjectPos().y + offs_y;
    double z =       getPos().z;

    geometry_msgs::Quaternion q;
    if (!computeOrientation(q))             return false;

    ROS_INFO("Going to: %g %g %g", x, y, z);
    if (!goToPose(x, y, z, q.x, q.y, q.z, q.w, "loose"))
    {
        return false;
    }

    if (!waitForCartEstData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int cnt_ik_fail = 0;

    ros::Rate r(THREAD_FREQ);

    while(RobotInterface::ok())
    {
        double new_elap_time = (ros::Time::now() - start_time).toSec();

        x = getObjectPos().x + offs_x;
        y = getObjectPos().y + offs_y;
        z = z_start - ARM_SPEED * new_elap_time / 1.3;

        ROS_DEBUG("Time %g Going to: %g %g %g Position: %g %g %g", new_elap_time, x, y, z,
                                                       getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,q.x, q.y, q.z, q.w))
        {
            cnt_ik_fail = 0;
            if (new_elap_time - elap_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            }
            elap_time = new_elap_time;

            if(hasCollidedIR("strict") || hasCollidedCD())
            {
                if (hasCollidedCD())
                {
                    moveArm("up", 0.002);
                }
                ROS_INFO("Collision!");
                return true;
            }

            r.sleep();
        }
        else    cnt_ik_fail++;

        if (cnt_ik_fail == 10)  return false;
    }

    return false;
}

bool ToolPicker::computeOffsets(double &_x_offs, double &_y_offs)
{
    if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        if (CartesianEstimatorClient::getObjectName() == "screwdriver")
        {
            _x_offs = +0.010;
            // _y_offs = +0.017;
        }
        else if (CartesianEstimatorClient::getObjectName() == "screws_box"  ||
                 CartesianEstimatorClient::getObjectName() == "brackets_box")
        {
            _x_offs = +0.06;
        }
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        if (CartesianEstimatorClient::getObjectName() == "screwdriver")
        {
            // _x_offs = -0.020;
            _y_offs = -0.010;
        }
        else if (CartesianEstimatorClient::getObjectName() == "screws_box"  ||
                 CartesianEstimatorClient::getObjectName() == "brackets_box")
        {
            _x_offs = +0.020;
            _y_offs = -0.058;
        }
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET, ACTION_GET_PASS or ACTION_CLEANUP!");
        return false;
    }

    return true;
}

bool ToolPicker::computeOrientation(geometry_msgs::Quaternion &_q)
{
    if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        quaternionFromDoubles(_q, POOL_ORI_R);
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        quaternionFromDoubles(_q, VERTICAL_ORI_R);
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET, ACTION_GET_PASS or ACTION_CLEANUP!");
        return false;
    }

    return true;
}

int ToolPicker::chooseObjectID(std::vector<int> _objs)
{
    if (getSubState() != CHECK_OBJ_IDS)
    {
        return ArmCtrl::chooseObjectID(_objs);
    }

    ROS_DEBUG("[%s] Choosing object IDs", getLimb().c_str());
    int res = -1;

    if (!waitForCartEstOK())
    {
        setSubState(NO_OBJ);
        return res;
    }

    if (!waitForCartEstObjsFound())
    {
        setSubState(NO_OBJ);
        return res;
    }

    std::vector<string> objs_str;
    for (size_t i = 0; i < _objs.size(); ++i)
    {
        objs_str.push_back(getObjectNameFromDB(_objs[i]));
    }

    std::vector<string> av_objects = getAvailableObjects(objs_str);

    if (av_objects.size() == 0)     return res;

    std::srand(std::time(0)); //use current time as seed
    string res_str = av_objects[rand() % av_objects.size()];

    res = getObjectIDFromDB(res_str);

    return res;
}

void ToolPicker::reduceSquish()
{
    XmlRpc::XmlRpcValue squish_params;
    // store the initial squish thresholds from the parameter server
    _n.getParam("/collision/"+getLimb()+"/baxter/squish_thresholds", squish_params);

    ROS_ASSERT_MSG(squish_params.getType()==XmlRpc::XmlRpcValue::TypeArray,
                          "[%s] Squish params is not an array! Type is %i",
                               getLimb().c_str(), squish_params.getType());

    for (int i = 0; i < squish_params.size(); ++i)
    {
        // store the initial squish thresholds for reset later
        squish_thresholds.push_back(squish_params[i]);
    }

    // adjust the squish thresholds for better tool picking
    squish_params[3] = 0.5 * static_cast<double>(squish_params[3]);
    squish_params[4] = 0.5 * static_cast<double>(squish_params[4]);
    squish_params[5] = 0.5 * static_cast<double>(squish_params[5]);

    ROS_INFO("[%s] Reduced squish thresholds for joint 3, 4 and 5 to %g. %g and %g .",
                            getLimb().c_str(), static_cast<double>(squish_params[3]),
                                               static_cast<double>(squish_params[5]),
                                               static_cast<double>(squish_params[5]));

    // set the squish thresholds in the parameter server to the new values
    _n.setParam("/collision/"+getLimb()+"/baxter/squish_thresholds", squish_params);
}

void ToolPicker::resetSquish()
{
    XmlRpc::XmlRpcValue squish_params;
    for (std::vector<int>::size_type i = 0; i != squish_thresholds.size(); i++)
    {
        // rewrite the squish parameters from the initial squish thresholds stored in reduceSquish()
        squish_params[i] = squish_thresholds[i];
    }

    ROS_INFO("[%s] Squish thresholds for joint 3, 4 and 5 set back to %g. %g and %g .",
                             getLimb().c_str(), static_cast<double>(squish_params[3]),
                                                static_cast<double>(squish_params[5]),
                                                static_cast<double>(squish_params[5]));

    // reset squish thresholds in the parameter server to the new values
    _n.setParam("/collision/"+getLimb()+"/baxter/squish_thresholds", squish_params);
}

void ToolPicker::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

void ToolPicker::setObjectID(int _obj)
{
    ArmCtrl::setObjectID(_obj);
    CartesianEstimatorClient::setObjectName(ArmCtrl::getObjectNameFromDB(_obj));
}

ToolPicker::~ToolPicker()
{
    resetSquish();
}

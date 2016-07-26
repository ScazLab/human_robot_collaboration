#include "actionProvider.h"

using namespace std;

actionProvider::actionProvider(std::string _name, std::string _limb) : name(_name)
{
    service = _n.advertiseService("/"+name+"/action_service",
                                  &actionProvider::serviceCallback, this);

    left_ctrl  = new ARTagController("left");
    // right_ctrl = new HoldController("right");
};

actionProvider::~actionProvider()
{
    if (left_ctrl)
    {
        delete left_ctrl;
        left_ctrl = 0;
    }
    
    // if (right_ctrl)
    // {
    //     delete right_ctrl;
    //     right_ctrl = 0;
    // }    
};

bool actionProvider::serviceCallback(baxter_collaboration::DoAction::Request  &req, 
                                     baxter_collaboration::DoAction::Response &res)
{
    string action = req.action;
    int    ID     = req.object;

    ROS_INFO("Service request received. Action: %s object: %i", action.c_str(), ID);
    res.success = false;

    if (action == ACTION_HOME)
    {
        res.success = left_ctrl -> goHome();
    }
    else if (action == ACTION_GET)
    {
        left_ctrl -> setAction(action);
        left_ctrl -> setMarkerID(ID);
        left_ctrl -> startInternalThread();

        while( int(left_ctrl->getState()) != PICK_UP  &&
               int(left_ctrl->getState()) != ERROR )
        {
            ros::spinOnce();
        }

        if (int(left_ctrl->getState() == PICK_UP ))
        {
            res.success = true;
        }
    }
    else if (action == ACTION_RELEASE)
    {
        res.success = left_ctrl -> releaseObject();
    }
    else if (action == ACTION_PASS)
    {
        left_ctrl -> setAction(action);
        left_ctrl -> startInternalThread();

        while( int(left_ctrl->getState()) != PASSED  &&
               int(left_ctrl->getState()) != ERROR )
        {
            ros::spinOnce();
        }

        if (int(left_ctrl->getState() == PASSED ))
        {
            res.success = true;
        }
    }
    else if (action == ACTION_HOLD)
    {
        // right_ctrl -> ...
    }
    else 
    {
        ROS_WARN("Requested action is not among the allowed types.");
        res.success = false;
    }

    ROS_INFO("Success: %i\n", res.success);
    return true;
};

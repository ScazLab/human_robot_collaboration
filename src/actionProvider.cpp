#include "actionProvider.h"

using namespace std;

actionProvider::actionProvider(std::string _name) : name(_name)
{
    service = _n.advertiseService("/"+name+"/action_service",
                                  &actionProvider::serviceCallback, this);

    left_ctrl  = new ARTagController("left");
    right_ctrl = new HoldController("right");

    // Move both arms to home
    left_ctrl -> setAction(ACTION_HOME);
    left_ctrl -> startInternalThread();

    right_ctrl -> setAction(ACTION_HOME);
    right_ctrl -> startInternalThread();

    while( (int(left_ctrl ->getState()) != START  &&
            int(left_ctrl ->getState()) != ERROR) ||
           (int(right_ctrl->getState()) != START  &&
            int(right_ctrl->getState()) != ERROR) )
    {
        ros::spinOnce();
    }

};

actionProvider::~actionProvider()
{
    if (left_ctrl)
    {
        delete left_ctrl;
        left_ctrl = 0;
    }

    if (right_ctrl)
    {
        delete right_ctrl;
        right_ctrl = 0;
    }    
};

bool actionProvider::serviceCallback(baxter_collaboration::DoAction::Request  &req, 
                                     baxter_collaboration::DoAction::Response &res)
{
    string action = req.action;
    int    ID     = req.object;

    ROS_INFO("Service request received. Action: %s object: %i", action.c_str(), ID);

    res.success = false;

    if (action == ACTION_HOME || action == ACTION_RELEASE)
    {
        left_ctrl -> setAction(action);
        left_ctrl -> startInternalThread();

        right_ctrl -> setAction(action);
        right_ctrl -> startInternalThread();

        while( (int(left_ctrl ->getState()) != START  &&
                int(left_ctrl ->getState()) != ERROR) ||
               (int(right_ctrl->getState()) != START  &&
                int(right_ctrl->getState()) != ERROR) )
        {
            ros::spinOnce();
        }

        if (int(left_ctrl ->getState()) == START && 
            int(right_ctrl->getState()) == START )
        {
            res.success = true;
        }
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

        if (int(left_ctrl->getState()) == PICK_UP )
        {
            res.success = true;
        }
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

        if (int(left_ctrl->getState()) == PASSED )
        {
            res.success = true;
        }
    }
    else if (action == ACTION_HOLD)
    {
        right_ctrl -> setAction(action);
        right_ctrl -> startInternalThread();

        while( int(right_ctrl->getState()) != PASSED  &&
               int(right_ctrl->getState()) != ERROR )
        {
            ros::spinOnce();
        }

        if (int(right_ctrl->getState()) == PASSED )
        {
            res.success = true;
        }
    }
    else 
    {
        ROS_WARN("Requested action is not among the allowed types.");
        res.success = false;
    }

    ROS_INFO("Service reply with success: %s\n", res.success?"true":"false");
    return true;
};

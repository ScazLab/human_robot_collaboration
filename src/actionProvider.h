#ifndef ACTION_PROVIDER_H
#define ACTION_PROVIDER_H

#include <string.h>

#include "robot_interface/artag_controller.h"
#include "robot_interface/hold_controller.h"

#include "baxter_collaboration/DoAction.h"

class actionProvider
{
private:
    std::string name;

    ros::NodeHandle _n;
    ros::ServiceServer service;

    ARTagController*  left_ctrl;
    HoldController*  right_ctrl;

public:
    actionProvider(std::string _name, std::string _limb);
    ~actionProvider();

    bool serviceCallback(baxter_collaboration::DoAction::Request  &req, 
                         baxter_collaboration::DoAction::Response &res);
};

#endif

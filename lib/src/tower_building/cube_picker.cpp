#include "tower_building/cube_picker.h"

using namespace std;
using namespace baxter_core_msgs;

CubePicker::CubePicker(std::string _name, std::string _limb, bool _no_robot) :
                       ARTagCtrl(_name, _limb, _no_robot)
{

}

void CubePicker::setHomeConfiguration()
{
    setHomeConf(0.1967, -0.8702, -1.0531,  1.5578,
                         0.6516,  1.2464, -0.1787);
}


CubePicker::~CubePicker()
{

}

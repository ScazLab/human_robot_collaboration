#ifndef __CUBE_PICKER_H__
#define __CUBE_PICKER_H__

#include <flatpack_furniture/artag_ctrl.h>

class CubePicker : public ARTagCtrl
{
private:
    /**
     * Home position with a specific joint configuration. This has
     * been introduced in order to force the arms to go to the home configuration
     * in always the same exact way, in order to clean the seed configuration in
     * case of subsequent inverse kinematics requests.
     *
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool homePoseStrict(bool disable_coll_av = false);
public:
    /**
     * Constructor
     */
    CubePicker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~CubePicker();
};

#endif

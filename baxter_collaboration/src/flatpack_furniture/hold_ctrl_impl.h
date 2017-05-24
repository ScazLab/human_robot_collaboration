#ifndef __HOLD_CTRL_IMPL_H__
#define __HOLD_CTRL_IMPL_H__

#include "flatpack_furniture/hold_ctrl.h"

class HoldCtrlImpl : public HoldCtrl
{
private:

public:
    /**
     * Constructor
     */
    HoldCtrlImpl(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~HoldCtrlImpl();
};

#endif

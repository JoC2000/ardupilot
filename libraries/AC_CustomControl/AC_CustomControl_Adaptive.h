#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AC_CustomControl_Simulink/Custom_Att_Controller.h>

class AC_CustomControl_Adaptive : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Adaptive(
        AC_CustomControl &frontend,
        AP_AHRS_View*& ahrs,
        AC_AttitudeControl*& att_control,
        AP_MotorsMulticopter*& motors,
        float dt);

    Vector3f update(void) override;
    void reset(void) override;
    
    Custom_Att_Controller simulinkn_controller;
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    float _dt;
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

#endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED

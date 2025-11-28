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
    AP_Float lambda_rm, lambda_pm, lambda_ym, lambda_s;
    AP_Float k1, k2, k3;
    AP_Float P1_11, P1_22, P2_11, P2_22, P3_11, P3_22;
    AP_Float sigma;
};

#endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Adaptive.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_Adaptive::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: Adaptive param1
    // @Description: Dummy parameter for Adaptive custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControl_Adaptive, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: Adaptive param2
    // @Description: Dummy parameter for Adaptive custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControl_Adaptive, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: Adaptive param3
    // @Description: Dummy parameter for Adaptive custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControl_Adaptive, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_Adaptive::AC_CustomControl_Adaptive(
    AC_CustomControl &frontend,
    AP_AHRS_View*& ahrs,
    AC_AttitudeControl*& att_control,
    AP_MotorsMulticopter*& motors,
    float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);

    simulinkn_controller.initialize();
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_Adaptive::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // run custom controller after here
     Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);

    attitude_target = _att_control->get_attitude_target_quat();
    Vector3f gyro_latest = _ahrs->get_gyro_latest();

    // '<Root>/x_reference'
    float arg_x_d[3]{attitude_target.get_euler_roll(), attitude_target.get_euler_pitch(), attitude_target.get_euler_yaw()};

    // '<Root>/dx_measured'
    float arg_d_x[3]{gyro_latest.x, gyro_latest.y, gyro_latest.z};

    // '<Root>/x_measured'
    float arg_x_real[3]{attitude_body.get_euler_roll(), attitude_body.get_euler_pitch(), attitude_body.get_euler_yaw()};

    // '<Root>/u_out'
    float arg_Out1[3];

    simulinkn_controller.step(arg_x_d, arg_d_x, arg_x_real, arg_Out1, _dt);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Adaptive custom controller working");

    // return what arducopter main controller outputted
    return Vector3f(arg_Out1[0], arg_Out1[1], arg_Out1[2]);
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_Adaptive::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_Adaptive_ENABLED

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
    AP_GROUPINFO("L_ROLL", 1, AC_CustomControl_Adaptive, lambda_rm, 25.45F),

    // @Param: PARAM2
    // @DisplayName: Adaptive param2
    // @Description: Dummy parameter for Adaptive custom controller backend
    // @User: Advanced
    AP_GROUPINFO("L_PITCH", 2, AC_CustomControl_Adaptive, lambda_pm, 25.45F),

    // @Param: PARAM3
    // @DisplayName: Adaptive param3
    // @Description: Dummy parameter for Adaptive custom controller backend
    // @User: Advanced
    AP_GROUPINFO("L_YAW", 3, AC_CustomControl_Adaptive, lambda_ym, 19.25F),

    AP_GROUPINFO("L_SLIDING", 4, AC_CustomControl_Adaptive, lambda_s, 1.5F),

    AP_GROUPINFO("K_ROLL", 5, AC_CustomControl_Adaptive, k1, 0.31F),

    AP_GROUPINFO("K_PITCH", 6, AC_CustomControl_Adaptive, k2, 0.31F),

    AP_GROUPINFO("K_YAW", 7, AC_CustomControl_Adaptive, k3, 0.21F),

    AP_GROUPINFO("P11_ROLL", 8, AC_CustomControl_Adaptive, P1_11, 0.35F),

    AP_GROUPINFO("P22_ROLL", 9, AC_CustomControl_Adaptive, P1_22, 0.15F),

    AP_GROUPINFO("P11_PITCH", 10, AC_CustomControl_Adaptive, P2_11, 0.35F),

    AP_GROUPINFO("P22_PITCH", 11, AC_CustomControl_Adaptive, P2_22, 0.15F),

    AP_GROUPINFO("P11_YAW", 12, AC_CustomControl_Adaptive, P3_11, 0.25F),

    AP_GROUPINFO("P22_YAW", 13, AC_CustomControl_Adaptive, P3_22, 0.15F),

    AP_GROUPINFO("SIGMA", 14, AC_CustomControl_Adaptive, sigma, 0.25F),

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
    float x_d[3]{attitude_target.get_euler_roll(), attitude_target.get_euler_pitch(), attitude_target.get_euler_yaw()};

    // '<Root>/dx_measured'
    float dx[3]{gyro_latest.x, gyro_latest.y, gyro_latest.z};

    // '<Root>/x_measured'
    float x[3]{attitude_body.get_euler_roll(), attitude_body.get_euler_pitch(), attitude_body.get_euler_yaw()};

    // '<Root>/u_out'
    float U[3];

    float lambdas[4]{lambda_rm.get(), lambda_pm.get(), lambda_ym.get(), lambda_s.get()};
    float k_gains[3]{k1.get(), k2.get(), k3.get()};
    float p_gains[6]{P1_11.get(),P1_22.get(),P2_11.get(),P2_22.get(),P3_11.get(),P3_22.get()};

    simulinkn_controller.step(x_d, dx, x, U, _dt, lambdas, k_gains, p_gains, sigma.get());

    // return what arducopter main controller outputted
    return Vector3f(U[0], U[1], U[2]);
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_Adaptive::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_Adaptive_ENABLED

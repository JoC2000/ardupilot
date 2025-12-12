#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Adaptive.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_Adaptive::var_info[] = {
    // @Param: L_ROLL
    // @DisplayName: L_ROLL
    // @Description: Lambda gain for Roll Reference Model
    // @User: Advanced
    AP_GROUPINFO("L_ROLL", 1, AC_CustomControl_Adaptive, lambda_rm, 30.15F),

    // @Param: L_PITCH
    // @DisplayName: L_PITCH
    // @Description: Lambda gain for Pitch Reference Model
    // @User: Advanced
    AP_GROUPINFO("L_PITCH", 2, AC_CustomControl_Adaptive, lambda_pm, 30.15F),

    // @Param: L_YAW
    // @DisplayName: L_YAW
    // @Description: Lambda gain for Yaw Reference Model
    // @User: Advanced
    AP_GROUPINFO("L_YAW", 3, AC_CustomControl_Adaptive, lambda_ym, 25.25F),

    // @Param: L_SLIDING
    // @DisplayName: L_SLIDING
    // @Description: Lambda gain for the sliding surface for error tracking
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING", 4, AC_CustomControl_Adaptive, lambda_s, 1.5F),

    // @Param: K_ROLL
    // @DisplayName: K_ROLL
    // @Description: K gain for Roll controller
    // @User: Advanced
    AP_GROUPINFO("K_ROLL", 5, AC_CustomControl_Adaptive, k1, 0.31F),

    // @Param: K_PITCH
    // @DisplayName: K_PITCH
    // @Description: K gain for Pitch controller
    // @User: Advanced
    AP_GROUPINFO("K_PITCH", 6, AC_CustomControl_Adaptive, k2, 0.31F),

    // @Param: K_YAW
    // @DisplayName: K_YAW
    // @Description: K gain for Yaw controller
    // @User: Advanced
    AP_GROUPINFO("K_YAW", 7, AC_CustomControl_Adaptive, k3, 0.21F),

    // @Param: P11_ROLL
    // @DisplayName: P11_ROLL
    // @Description: P1 roll adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P11_ROLL", 8, AC_CustomControl_Adaptive, P1_11, 0.20F),

    // @Param: P22_ROLL
    // @DisplayName: P22_ROLL
    // @Description: P2 roll adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P22_ROLL", 9, AC_CustomControl_Adaptive, P1_22, 0.15F),

    // @Param: P11_PITCH
    // @DisplayName: P11_PITCH
    // @Description: P1 pitch adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P11_PITCH", 10, AC_CustomControl_Adaptive, P2_11, 0.15F),

    // @Param: P22_PITCH
    // @DisplayName: P22_PITCH
    // @Description: P2 pitch adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P22_PITCH", 11, AC_CustomControl_Adaptive, P2_22, 0.15F),

    // @Param: P11_YAW
    // @DisplayName: P11_YAW
    // @Description: P1 yaw adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P11_YAW", 12, AC_CustomControl_Adaptive, P3_11, 0.10F),

    // @Param: P22_YAW
    // @DisplayName: P22_YAW
    // @Description: P2 yaw adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P22_YAW", 13, AC_CustomControl_Adaptive, P3_22, 0.10F),

    // @Param: SIGMA
    // @DisplayName: SIGMA
    // @Description: Gain for sigma modification rule
    // @User: Advanced
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

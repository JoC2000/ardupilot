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
    AP_GROUPINFO("L_ROLL", 1, AC_CustomControl_Adaptive, lambda_pm, 30.15F),

    // @Param: L_PITCH
    // @DisplayName: L_PITCH
    // @Description: Lambda gain for Pitch Reference Model
    // @User: Advanced
    AP_GROUPINFO("L_PITCH", 2, AC_CustomControl_Adaptive, lambda_qm, 30.15F),

    // @Param: L_YAW
    // @DisplayName: L_YAW
    // @Description: Lambda gain for Yaw Reference Model
    // @User: Advanced
    AP_GROUPINFO("L_YAW", 3, AC_CustomControl_Adaptive, lambda_rm, 25.25F),

    // @Param: LP_SLIDING
    // @DisplayName: L_SLIDING
    // @Description: Lambda gain for x rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("LP_SLIDING", 4, AC_CustomControl_Adaptive, lambda_ps, 1.5F),

    // @Param: LQ_SLIDING
    // @DisplayName: L_SLIDING
    // @Description: Lambda gain for y rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("LQ_SLIDING", 5, AC_CustomControl_Adaptive, lambda_qs, 1.5F),

    // @Param: LR_SLIDING
    // @DisplayName: LR_SLIDING
    // @Description: Lambda gain for z rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("LR_SLIDING", 6, AC_CustomControl_Adaptive, lambda_rs, 1.5F),

    // @Param: K_ROLL
    // @DisplayName: K_ROLL
    // @Description: K gain for Roll controller
    // @User: Advanced
    AP_GROUPINFO("K_ROLL", 7, AC_CustomControl_Adaptive, k1, 0.31F),

    // @Param: K_PITCH
    // @DisplayName: K_PITCH
    // @Description: K gain for Pitch controller
    // @User: Advanced
    AP_GROUPINFO("K_PITCH", 8, AC_CustomControl_Adaptive, k2, 0.31F),

    // @Param: K_YAW
    // @DisplayName: K_YAW
    // @Description: K gain for Yaw controller
    // @User: Advanced
    AP_GROUPINFO("K_YAW", 9, AC_CustomControl_Adaptive, k3, 0.21F),

    // @Param: P_ROLL
    // @DisplayName: P_ROLL
    // @Description: P roll adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_ROLL", 10, AC_CustomControl_Adaptive, P_11, 0.20F),

    // @Param: P_PITCH
    // @DisplayName: P_PITCH
    // @Description: P pitch adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_PITCH", 11, AC_CustomControl_Adaptive, P_22, 0.15F),

    // @Param: P_YAW
    // @DisplayName: P_YAW
    // @Description: P yaw adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_YAW", 12, AC_CustomControl_Adaptive, P_33, 0.10F),

    // @Param: ROLL_G
    // @DisplayName: ROLL_G
    // @Description: Initial guess for roll intertia
    // @User: Advanced
    AP_GROUPINFO("ROLL_G", 13, AC_CustomControl_Adaptive, ah_guess_r, 0.03F),

    // @Param: PITCH_G
    // @DisplayName: PITCH_G
    // @Description: Initial guess for pitch intertia
    // @User: Advanced
    AP_GROUPINFO("PITCH_G", 14, AC_CustomControl_Adaptive, ah_guess_p, 0.02F),

    // @Param: YAW_G
    // @DisplayName: YAW_G
    // @Description: Initial guess for yaw intertia
    // @User: Advanced
    AP_GROUPINFO("YAW_G", 15, AC_CustomControl_Adaptive, ah_guess_y, 0.01F),

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
    

    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, 
                                                 _thrust_angle, _thrust_error_angle);

    Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    Vector3f target_rate;
    target_rate[0] = ang_vel_body_feedforward[0];
    target_rate[1] = ang_vel_body_feedforward[1];
    target_rate[2] = ang_vel_body_feedforward[2];
    
    Vector3f lambdas_model{lambda_rm.get(), lambda_qm.get(), lambda_rm.get()};
    Vector3f lambdas_sliding{lambda_ps.get(), lambda_qs.get(), lambda_rs.get()};
    Vector3f k_gains{k1.get(), k2.get(), k3.get()};
    Vector3f p_gains{P_11.get(), P_22.get(), P_33.get()};
    Vector3f guesses{ah_guess_r.get(), ah_guess_p.get(), ah_guess_y.get()};
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;

    simulinkn_controller.step(target_rate, gyro_latest, motor_out, attitude_error, _dt, lambdas_model, lambdas_sliding, k_gains, p_gains, guesses);

    // return what arducopter main controller outputted
    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_Adaptive::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_Adaptive_ENABLED

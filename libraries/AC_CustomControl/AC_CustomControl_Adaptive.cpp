#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Adaptive.h"

#include <GCS_MAVLink/GCS.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_Adaptive::var_info[] = {
    // @Param: AH_MIN_R
    // @DisplayName: AH_MIN_R
    // @Description: Min inertial estimation for roll angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MIN_R", 1, AC_CustomControl_Adaptive, ah_min_r, 0.01F),

    // @Param: AH_MIN_P
    // @DisplayName: AH_MIN_P
    // @Description: Min inertial estimation for pitch angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MIN_P", 2, AC_CustomControl_Adaptive, ah_min_p, 0.01F),

    // @Param: AH_MIN_Y
    // @DisplayName: AH_MIN_Y
    // @Description: Min inertial estimation for yaw angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MIN_Y", 3, AC_CustomControl_Adaptive, ah_min_y, 0.01F),

    // @Param: AH_MAX_R
    // @DisplayName: AH_MAX_R
    // @Description: Max inertial estimation for roll angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MAX_R", 4, AC_CustomControl_Adaptive, ah_max_r, 0.1F),

    // @Param: AH_MAX_P
    // @DisplayName: AH_MAX_P
    // @Description: Max inertial estimation for pitch angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MAX_P", 5, AC_CustomControl_Adaptive, ah_max_p, 0.1F),

    // @Param: AH_MAX_Y
    // @DisplayName: AH_MAX_Y
    // @Description: Max inertial estimation for yaw angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MAX_Y", 6, AC_CustomControl_Adaptive, ah_max_y, 0.1F),

    // @Param: L_SLIDING_R
    // @DisplayName: L_SLIDING_R
    // @Description: Lambda gain for x rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING_R", 7, AC_CustomControl_Adaptive, lambda_sr, 0.87F),

    // @Param: L_SLIDING_P
    // @DisplayName: L_SLIDING_P
    // @Description: Lambda gain for y rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING_P", 8, AC_CustomControl_Adaptive, lambda_sp, 0.73F),

    // @Param: L_SLIDING_Y
    // @DisplayName: L_SLIDING_Y
    // @Description: Lambda gain for z rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING_Y", 9, AC_CustomControl_Adaptive, lambda_sy, 0.45F),

    // @Param: K_ROLL
    // @DisplayName: K_ROLL
    // @Description: K gain for Roll controller
    // @User: Advanced
    AP_GROUPINFO("K_ROLL", 10, AC_CustomControl_Adaptive, k1, 0.045F),

    // @Param: K_PITCH
    // @DisplayName: K_PITCH
    // @Description: K gain for Pitch controller
    // @User: Advanced
    AP_GROUPINFO("K_PITCH", 11, AC_CustomControl_Adaptive, k2, 0.045F),

    // @Param: K_YAW
    // @DisplayName: K_YAW
    // @Description: K gain for Yaw controller
    // @User: Advanced
    AP_GROUPINFO("K_YAW", 12, AC_CustomControl_Adaptive, k3, 0.023F),

    // @Param: P_ROLL
    // @DisplayName: P_ROLL
    // @Description: P roll adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_ROLL", 13, AC_CustomControl_Adaptive, p_roll, 0.08F),

    // @Param: P_PITCH
    // @DisplayName: P_PITCH
    // @Description: P pitch adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_PITCH", 14, AC_CustomControl_Adaptive, p_pitch, 0.08F),

    // @Param: P_YAW
    // @DisplayName: P_YAW
    // @Description: P yaw adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_YAW", 15, AC_CustomControl_Adaptive, p_yaw, 0.05F),

    // @Param: GUESS_R
    // @DisplayName: GUESS_R
    // @Description: Initial guess for roll intertia
    // @User: Advanced
    AP_GROUPINFO("GUESS_R", 16, AC_CustomControl_Adaptive, ah_guess_r, 0.01F),

    // @Param: GUESS_P
    // @DisplayName: GUESS_P
    // @Description: Initial guess for pitch intertia
    // @User: Advanced
    AP_GROUPINFO("GUESS_P", 17, AC_CustomControl_Adaptive, ah_guess_p, 0.01F),

    // @Param: GUESS_Y
    // @DisplayName: GUESS_Y
    // @Description: Initial guess for yaw intertia
    // @User: Advanced
    AP_GROUPINFO("GUESS_Y", 18, AC_CustomControl_Adaptive, ah_guess_y, 0.01F),

    // @Param: L_MODEL_R
    // @DisplayName: L_MODEL_R
    // @Description: Lambda gain for x rate reference model
    // @User: Advanced
    AP_GROUPINFO("L_MODEL_R", 19, AC_CustomControl_Adaptive, lambda_mr, 15.0F),

    // @Param: L_MODEL_P
    // @DisplayName: L_MODEL_P
    // @Description: Lambda gain for y rate reference model
    // @User: Advanced
    AP_GROUPINFO("L_MODEL_P", 20, AC_CustomControl_Adaptive, lambda_mp, 15.0F),

    // @Param: L_MODEL_Y
    // @DisplayName: L_MODEL_Y
    // @Description: Lambda gain for z rate reference model
    // @User: Advanced
    AP_GROUPINFO("L_MODEL_Y", 21, AC_CustomControl_Adaptive, lambda_my, 15.0F),

    // @Param: P_ROLL_D
    // @DisplayName: P_ROLL_D
    // @Description: Adaptive gain for roll drag effects
    // @User: Advanced
    AP_GROUPINFO("P_ROLL_D", 22, AC_CustomControl_Adaptive, p_roll_d, 0.08F),

    // @Param: P_PITCH_D
    // @DisplayName: P_PITCH_D
    // @Description: Adaptive gain for pitch drag effects
    // @User: Advanced
    AP_GROUPINFO("P_PITCH_D", 23, AC_CustomControl_Adaptive, p_pitch_d, 0.08F),

    // @Param: P_YAW_D
    // @DisplayName: P_YAW_D
    // @Description: Adaptive gain for yaw drag effects
    // @User: Advanced
    AP_GROUPINFO("P_YAW_D", 24, AC_CustomControl_Adaptive, p_yaw_d, 0.03F),

    // @Param: DH_MIN_R
    // @DisplayName: DH_MIN_R
    // @Description: Min roll drag estimation for drag effects
    // @User: Advanced
    AP_GROUPINFO("DH_MIN_R", 25, AC_CustomControl_Adaptive, dh_min_r, 0.00F),

    // @Param: DH_MIN_P
    // @DisplayName: DH_MIN_P
    // @Description: Min pitch drag estimation for drag effects
    // @User: Advanced
    AP_GROUPINFO("DH_MIN_P", 26, AC_CustomControl_Adaptive, dh_min_p, 0.00F),

    // @Param: DH_MIN_Y
    // @DisplayName: DH_MIN_Y
    // @Description: Min yaw drag estimation for drag effects
    // @User: Advanced
    AP_GROUPINFO("DH_MIN_Y", 27, AC_CustomControl_Adaptive, dh_min_y, 0.00F),

    // @Param: DH_MAX_R
    // @DisplayName: DH_MAX_R
    // @Description: Max roll drag estimation for drag effects
    // @User: Advanced
    AP_GROUPINFO("DH_MAX_R", 28, AC_CustomControl_Adaptive, dh_max_r, 0.1F),

    // @Param: DH_MAX_P
    // @DisplayName: DH_MAX_P
    // @Description: Max pitch drag estimation for drag effects
    // @User: Advanced
    AP_GROUPINFO("DH_MAX_P", 29, AC_CustomControl_Adaptive, dh_max_p, 0.1F),

    // @Param: DH_MAX_Y
    // @DisplayName: DH_MAX_Y
    // @Description: Max yaw drag estimation for drag effects
    // @User: Advanced
    AP_GROUPINFO("DH_MAX_Y", 30, AC_CustomControl_Adaptive, dh_max_y, 0.1F),

    // @Param: D_GUESS_R
    // @DisplayName: D_GUESS_R
    // @Description: Initial guess for roll drag effects
    // @User: Advanced
    AP_GROUPINFO("D_GUESS_R", 31, AC_CustomControl_Adaptive, dh_guess_r, 0.01F),

    // @Param: D_GUESS_P
    // @DisplayName: D_GUESS_P
    // @Description: Initial guess for pitch drag effects
    // @User: Advanced
    AP_GROUPINFO("D_GUESS_P", 32, AC_CustomControl_Adaptive, dh_guess_p, 0.01F),

    // @Param: D_GUESS_Y
    // @DisplayName: D_GUESS_Y
    // @Description: Initial guess for yaw drag effects
    // @User: Advanced
    AP_GROUPINFO("D_GUESS_Y", 33, AC_CustomControl_Adaptive, dh_guess_y, 0.01F),

    // @Param: P_ROLL_B
    // @DisplayName: P_ROLL_B
    // @Description: Adaptive gain for roll constant effects
    // @User: Advanced
    AP_GROUPINFO("P_ROLL_B", 34, AC_CustomControl_Adaptive, p_roll_b, 0.01F),

    // @Param: P_PITCH_B
    // @DisplayName: P_PITCH_B
    // @Description: Adaptive gain for pitch constant effects
    // @User: Advanced
    AP_GROUPINFO("P_PITCH_B", 35, AC_CustomControl_Adaptive, p_pitch_b, 0.01F),

    // @Param: P_YAW_B
    // @DisplayName: P_YAW_B
    // @Description: Adaptive gain for yaw constant effects
    // @User: Advanced
    AP_GROUPINFO("P_YAW_B", 36, AC_CustomControl_Adaptive, p_yaw_b, 0.005F),

    // @Param: BH_MIN_R
    // @DisplayName: BH_MIN_R
    // @Description: Min roll estimation for constant effects
    // @User: Advanced
    AP_GROUPINFO("BH_MIN_R", 37, AC_CustomControl_Adaptive, bh_min_r, -0.5F),

    // @Param: BH_MIN_P
    // @DisplayName: BH_MIN_P
    // @Description: Min pitch estimation for constant effects
    // @User: Advanced
    AP_GROUPINFO("BH_MIN_P", 38, AC_CustomControl_Adaptive, bh_min_p, -0.5F),

    // @Param: BH_MIN_Y
    // @DisplayName: BH_MIN_Y
    // @Description: Min yaw estimation for constant effects
    // @User: Advanced
    AP_GROUPINFO("BH_MIN_Y", 39, AC_CustomControl_Adaptive, bh_min_y, -0.5F),

    // @Param: BH_MAX_R
    // @DisplayName: BH_MAX_R
    // @Description: Max roll estimation for constant effects
    // @User: Advanced
    AP_GROUPINFO("BH_MAX_R", 40, AC_CustomControl_Adaptive, bh_max_r, 0.5F),

    // @Param: BH_MAX_P
    // @DisplayName: BH_MAX_P
    // @Description: Max pitch estimation for constant effects
    // @User: Advanced
    AP_GROUPINFO("BH_MAX_P", 41, AC_CustomControl_Adaptive, bh_max_p, 0.5F),

    // @Param: BH_MAX_Y
    // @DisplayName: BH_MAX_Y
    // @Description: Max yaw estimation for constant effects
    // @User: Advanced
    AP_GROUPINFO("BH_MAX_Y", 42, AC_CustomControl_Adaptive, bh_max_y, 0.5F),

    // @Param: B_GUESS_R
    // @DisplayName: B_GUESS_R
    // @Description: Initial guess for roll constant effects
    // @User: Advanced
    AP_GROUPINFO("B_GUESS_R", 43, AC_CustomControl_Adaptive, bh_guess_r, 0.0F),

    // @Param: B_GUESS_P
    // @DisplayName: B_GUESS_P
    // @Description: Initial guess for pitch constant effects
    // @User: Advanced
    AP_GROUPINFO("B_GUESS_P", 44, AC_CustomControl_Adaptive, bh_guess_p, 0.0F),

    // @Param: B_GUESS_Y
    // @DisplayName: B_GUESS_Y
    // @Description: Initial guess for yaw constant effects
    // @User: Advanced
    AP_GROUPINFO("B_GUESS_Y", 45, AC_CustomControl_Adaptive, bh_guess_y, 0.0F),

    AP_SUBGROUPINFO(_p_angle_roll,  "ANG_RLL_", 46, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 47, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_angle_yaw,   "ANG_YAW_", 48, AC_CustomControl_Adaptive, AC_P),

    // PID baseline — rate PIDs
    AP_SUBGROUPINFO(_pid_rate_roll,  "RAT_RLL_", 49, AC_CustomControl_Adaptive, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 50, AC_CustomControl_Adaptive, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw,   "RAT_YAW_", 51, AC_CustomControl_Adaptive, AC_PID),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_Adaptive::AC_CustomControl_Adaptive(
    AC_CustomControl &frontend,
    AP_AHRS_View*& ahrs,
    AC_AttitudeControl*& att_control,
    AP_MotorsMulticopter*& motors,
    float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P * 0.9F),
    _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P * 0.9F),
    _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P * 0.9F),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P * 0.9F, AC_ATC_MULTI_RATE_RP_I * 0.9F, AC_ATC_MULTI_RATE_RP_D * 0.9F, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.9F, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.9F, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.9F),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P * 0.9F, AC_ATC_MULTI_RATE_RP_I * 0.9F, AC_ATC_MULTI_RATE_RP_D * 0.9F, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.9F, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.9F, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.9F),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P * 0.9F, AC_ATC_MULTI_RATE_YAW_I * 0.9F, AC_ATC_MULTI_RATE_YAW_D * 0.9F, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX * 0.9F, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.9F, AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.9F, 0.0f)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
    adaptive_controller.initialize();
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
    target_rate[0] = _p_angle_roll.kP() * attitude_error.x + ang_vel_body_feedforward[0];
    target_rate[1] = _p_angle_pitch.kP() * attitude_error.y + ang_vel_body_feedforward[1];
    target_rate[2] = _p_angle_yaw.kP() * attitude_error.z + ang_vel_body_feedforward[2];

    Vector3f ah_min{ah_min_r.get(), ah_min_p.get(), ah_min_y.get()};
    Vector3f ah_max{ah_max_r.get(), ah_max_p.get(), ah_max_y.get()};
    Vector3f dh_min{dh_min_r.get(), dh_min_p.get(), dh_min_y.get()};
    Vector3f dh_max{dh_max_r.get(), dh_max_p.get(), dh_max_y.get()};
    Vector3f bh_min{bh_min_r.get(), bh_min_p.get(), bh_min_y.get()};
    Vector3f bh_max{bh_max_r.get(), bh_max_p.get(), bh_max_y.get()};
    Vector3f lambdas_sliding{lambda_sr.get(), lambda_sp.get(), lambda_sy.get()};
    Vector3f lambdas_model{lambda_mr.get(), lambda_mp.get(), lambda_my.get()};
    Vector3f kd_gains{k1.get(), k2.get(), k3.get()};
    Vector3f p_gains{p_roll.get(), p_pitch.get(), p_yaw.get()};
    Vector3f p_gains_d{p_roll_d.get(), p_pitch_d.get(), p_yaw_d.get()};
    Vector3f p_gains_b{p_roll_b.get(), p_pitch_b.get(), p_yaw_b.get()};
    
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    Vector3f U_pid;

    U_pid.x = _pid_rate_roll.update_all(target_rate[0], gyro_latest[0], _dt, false);
    U_pid.y = _pid_rate_pitch.update_all(target_rate[1], gyro_latest[1], _dt, false);
    U_pid.z = _pid_rate_yaw.update_all(target_rate[2], gyro_latest[2], _dt, false);

    Vector3f U_adaptive;
    adaptive_controller.step(
                            target_rate, gyro_latest, U_adaptive, _dt,
                            ah_min, ah_max, lambdas_model, kd_gains,
                            p_gains, p_gains_d, dh_min, dh_max, p_gains_b, bh_min, bh_max);

    Vector3f U_total = U_pid + U_adaptive;

    adaptive_controller.Log_CC0(U_total, U_pid, U_adaptive, attitude_error);

    return U_total;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_Adaptive::reset(void)
{
    Vector3f guesses_ah{ah_guess_r.get(), ah_guess_p.get(), ah_guess_y.get()};
    Vector3f guesses_dh{dh_guess_r.get(), dh_guess_p.get(), dh_guess_y.get()};
    Vector3f guesses_bh{bh_guess_r.get(), bh_guess_p.get(), bh_guess_y.get()};
    adaptive_controller.reset_ah(guesses_ah, guesses_dh, guesses_bh);

    _pid_rate_roll.reset_I();
    _pid_rate_pitch.reset_I();
    _pid_rate_yaw.reset_I();
    _pid_rate_roll.reset_filter();
    _pid_rate_pitch.reset_filter();
    _pid_rate_yaw.reset_filter();
}

void AC_CustomControl_Adaptive::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
    _pid_rate_yaw.set_notch_sample_rate(sample_rate);
#endif
}

#endif  // AP_CUSTOMCONTROL_Adaptive_ENABLED

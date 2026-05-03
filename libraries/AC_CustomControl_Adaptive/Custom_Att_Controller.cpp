#include "Custom_Att_Controller.h"

void Custom_Att_Controller::Log_CC0(Vector3f U, Vector3f control, Vector3f adapt, Vector3f error) const
{
    struct log_CC0 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC0_MSG),
time_us     : AP_HAL::micros64(),
u_roll      : U.x,
u_pitch     : U.y,
u_yaw       : U.z,
control_r   : control.x,
control_p   : control.y,
control_y   : control.z,
adapt_r     : adapt.x,
adapt_p     : adapt.y,
adapt_y     : adapt.z,
err1        : degrees(error.x),
err2        : degrees(error.y),
err3        : degrees(error.z)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC1(Vector3f wr, Vector3f dwr, Vector3f wm, Vector3f d_wm) const
{
    struct log_CC1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC1_MSG),
time_us     : AP_HAL::micros64(),
wr1         : degrees(wr.x),
wr2         : degrees(wr.y),
wr3         : degrees(wr.z),
dwr1        : degrees(dwr.x),
dwr2        : degrees(dwr.y),
dwr3        : degrees(dwr.z),
wm1         : degrees(wm.x),
wm2         : degrees(wm.y),
wm3         : degrees(wm.z),
dwm1        : degrees(d_wm.x),
dwm2        : degrees(d_wm.y),
dwm3        : degrees(d_wm.z)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC2(Vector3f w_, Vector3f wd, Vector3f s_, Vector3f ys_) const
{
    struct log_CC2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC2_MSG),
time_us     : AP_HAL::micros64(),
w1          : degrees(w_.x),
w2          : degrees(w_.y),
w3          : degrees(w_.z),
wd1         : degrees(wd.x),
wd2         : degrees(wd.y),
wd3         : degrees(wd.z),
s_roll      : degrees(s_.x),
s_pitch     : degrees(s_.y),
s_yaw       : degrees(s_.z),
ys1         : ys_.x,
ys2         : ys_.y,
ys3         : ys_.z
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC3(Vector3f ah, Vector3f dah, Vector3f dh, Vector3f ddh) const
{
    struct log_CC3 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC3_MSG),
time_us     : AP_HAL::micros64(),
ah1         : ah.x,
ah2         : ah.y,
ah3         : ah.z,
dah1        : dah.x,
dah2        : dah.y,
dah3        : dah.z,
dh1         : dh.x,
dh2         : dh.y,
dh3         : dh.z,
ddh1        : ddh.x,
ddh2        : ddh.y,
ddh3        : ddh.z,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC4(Vector3f b_h, Vector3f d_bh) const
{
    struct log_CC4 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC4_MSG),
        time_us : AP_HAL::micros64(),
        bh1     : b_h.x,
        bh2     : b_h.y,
        bh3     : b_h.z,
        dbh1    : d_bh.x,
        dbh2    : d_bh.y,
        dbh3    : d_bh.z
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

float Custom_Att_Controller::param_projection(float ahat, float dahat, float ahat_min, float ahat_max)
{
    if (ahat >= ahat_max && dahat > 0.0F) {
        return 0.0F;
    } else if (ahat <= ahat_min && dahat < 0.0F) {
        return 0.0F;
    } else {
        return dahat;
    }
}

void Custom_Att_Controller::step(
    Vector3f w_d, Vector3f w, Vector3f &U, Vector3f att_error, float dt, Vector3f ah_min,
    Vector3f ah_max, Vector3f lambdas_model, Vector3f lambdas_sliding, Vector3f kd_gains, 
    Vector3f p_gains, Vector3f p_gains_d, Vector3f dh_min, Vector3f dh_max, Vector3f p_gains_b, Vector3f bh_min, Vector3f bh_max)
{
    Y.zero();

    // Reference model
    dw_m = w_d - w_m;
    dw_m *= lambdas_model;
    w_m += dw_m * dt;

    // Virtual controller reference, includes velocity and attitude targets
    // xr = x_m + (lambda * error)
    Vector3f ls_att = att_error;
    ls_att *= lambdas_sliding;
    w_r = w_m + ls_att;

    // Derivate of the virtual reference, the acceleration reference of the reference model
    // dxr = dx_m + (lambda * derror) 
    Vector3f ls_diff = w_m - w;
    ls_diff *= lambdas_sliding;
    dw_r = dw_m + ls_diff;

    // Sliding surface
    // Desired - Actual to match ArduPilot's logic
    s_last_ = s_filt_;

    s = w_r - w;

    s_filt_ += (s - s_last_) * calc_lowpass_alpha_dt(dt, 15.0F);

    // Populate Y matrix
    Y.a.x = dw_r.x;         Y.a.y = -(w.y * w.z);   Y.a.z = w.y * w.z;
    Y.b.x = w.x * w.z;      Y.b.y = dw_r.y;         Y.b.z = -(w.x * w.z);
    Y.c.x = -(w.x * w.y);   Y.c.y = w.x * w.y;      Y.c.z = dw_r.z;

    // Populate Yd matrix
    // Yd matrix is a diagonal matrix that contains wx,wy and wz.
    // Instead of creating a diagonal matrix I am just going to reuse the w vector instead.
    // Y_d = w;

    // Adaptation contribution
    Vector3f drag_adaptation = w;
    drag_adaptation *= d_hat;

    // Sum of system and drag adaptation
    adaptation = Y * a_hat + drag_adaptation;

    // Controller contribution Kd * s
    controller = s_filt_;
    controller *= kd_gains;

    // Control output
    U = adaptation + controller + b_hat;

    // Adaptation law
    // da_hat = P*Y^(T)*s
    ys = Y.transposed() * s_filt_;
    da_hat = ys;
    da_hat *= p_gains;

    // Adaptation for non linear effects
    // dd_hat = P*Y_d^(T)*s
    dd_hat = s_filt_;
    dd_hat *= w;
    dd_hat *= p_gains_d;

    // Adaptation for constant disturbance
    // db_hat = P*Y_b(T)*s (Regressor here is identity matrix)
    db_hat = s_filt_;
    db_hat *= p_gains_b; 

    // Apply param projection to stop adaptation if not necessary
    da_hat.x = param_projection(a_hat.x, da_hat.x, ah_min.x, ah_max.x);
    da_hat.y = param_projection(a_hat.y, da_hat.y, ah_min.y, ah_max.y);
    da_hat.z = param_projection(a_hat.z, da_hat.z, ah_min.z, ah_max.z);

    dd_hat.x = param_projection(d_hat.x, dd_hat.x, dh_min.x, dh_max.x);
    dd_hat.y = param_projection(d_hat.y, dd_hat.y, dh_min.y, dh_max.y);
    dd_hat.z = param_projection(d_hat.z, dd_hat.z, dh_min.z, dh_max.z);

    db_hat.x = param_projection(b_hat.x, db_hat.x, bh_min.x, bh_max.x);
    db_hat.y = param_projection(b_hat.y, db_hat.y, bh_min.y, bh_max.y);
    db_hat.z = param_projection(b_hat.z, db_hat.z, bh_min.z, bh_max.z);

    // Update adaptation
    a_hat += da_hat * dt;
    a_hat.x = constrain_float(a_hat.x, ah_min.x, ah_max.x);
    a_hat.y = constrain_float(a_hat.y, ah_min.y, ah_max.y);
    a_hat.z = constrain_float(a_hat.z, ah_min.z, ah_max.z);

    d_hat += dd_hat * dt;
    d_hat.x = constrain_float(d_hat.x, dh_min.x, dh_max.x);
    d_hat.y = constrain_float(d_hat.y, dh_min.y, dh_max.y);
    d_hat.z = constrain_float(d_hat.z, dh_min.z, dh_max.z);

    b_hat += db_hat * dt;
    b_hat.x = constrain_float(b_hat.x, bh_min.x, bh_max.x);
    b_hat.y = constrain_float(b_hat.y, bh_min.y, bh_max.y);
    b_hat.z = constrain_float(b_hat.z, bh_min.z, bh_max.z);

    // Log debug variables
    Log_CC0(U, controller, adaptation, att_error);
    Log_CC1(w_r, dw_r, w_m, dw_m);
    Log_CC2(w, w_d, s, ys);
    Log_CC3(a_hat, da_hat, d_hat, dd_hat);
    Log_CC4(b_hat, db_hat);
}

void Custom_Att_Controller::initialize()
{
    w_r.zero();
    dw_r.zero();
    w_m.zero();
    dw_m.zero();
    s.zero();
    Y.zero();
    da_hat.zero();
    dd_hat.zero();
    db_hat.zero();
    controller.zero();
    adaptation.zero();
    s_filt_.zero();
    s_last_.zero();
}

void Custom_Att_Controller::reset_ah(Vector3f guesses_ah, Vector3f guesses_dh, Vector3f guesses_bh)
{
    a_hat = guesses_ah;
    d_hat = guesses_dh;
    b_hat = guesses_bh;
    s_filt_.zero();
    s_last_.zero();
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() {}

Custom_Att_Controller::~Custom_Att_Controller() = default;
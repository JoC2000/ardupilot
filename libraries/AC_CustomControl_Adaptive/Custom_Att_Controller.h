#pragma once
#include <cmath>
#include <AP_Logger/AP_Logger.h>

// Class declaration for model Custom_Att_Controller
class Custom_Att_Controller final
{
    // public data and function members
public:
    // Copy Constructor
    Custom_Att_Controller(Custom_Att_Controller const&) = delete;

    // Assignment Operator
    Custom_Att_Controller& operator= (Custom_Att_Controller const&) & = delete;

    // Move Constructor
    Custom_Att_Controller(Custom_Att_Controller &&) = delete;

    // Move Assignment Operator
    Custom_Att_Controller& operator= (Custom_Att_Controller &&) = delete;

    // Controller initialization
    void initialize();

    // Controller step function
    void step(Vector3f w_d, Vector3f w, Vector3f &U, Vector3f att_error, float dt,
              Vector3f ah_min, Vector3f ah_max, Vector3f lambdas_model, Vector3f lambdas_sliding,
              Vector3f kd_gains, Vector3f p_gains, Vector3f p_gains_d, Vector3f dh_min, Vector3f dh_max, Vector3f p_gains_b ,Vector3f bh_min, Vector3f bh_max);

    void Log_CC0(Vector3f U, Vector3f controller, Vector3f adaptation, Vector3f att_error) const;

    void Log_CC1(Vector3f wr, Vector3f d_wr, Vector3f wm, Vector3f dwm) const;

    void Log_CC2(Vector3f w, Vector3f wd, Vector3f s, Vector3f ys) const;

    void Log_CC3(Vector3f ah, Vector3f dah, Vector3f dh, Vector3f ddh) const;

    void Log_CC4(Vector3f bh, Vector3f dbh) const;

    float param_projection(float a_hat, float da_hat, float ahat_min, float ahat_max);

    void reset_ah(Vector3f guesses_ah, Vector3f guesses_dh, Vector3f guesses_bh);

    // Constructor
    Custom_Att_Controller();

    // Destructor
    ~Custom_Att_Controller();

    // private data and function members
private:
    Vector3f a_hat, da_hat;
    Vector3f b_hat, db_hat;
    Vector3f d_hat, dd_hat;
    Vector3f w_r, dw_r;
    Vector3f dw_m, w_m;
    Vector3f s, ys;
    Vector3f s_filt_, s_last_;
    Vector3f controller, adaptation;
    Matrix3f Y;
};
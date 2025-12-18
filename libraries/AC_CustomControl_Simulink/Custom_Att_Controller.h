#pragma once
#include <cmath>
#include <AP_Logger/AP_Logger.h>
// Class declaration for model Custom_Att_Controller
class Custom_Att_Controller final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_Custom_Att_Controller_T {
    float ah[6];                              // 'ah'
    float x_m[3];                             // 'x_m integrator'
    float dx_m[3];                            // 'dx_m integrator'
    unsigned char x_m_IC_LOADING;             // 'x_m initial condition'
    unsigned char dx_m_integrator_IC_LOADING; // 'dx_m initial condition'
  };

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
  void step(Vector3f wd, Vector3f w, Vector3f &U, Vector3f att_error, float dt,
            Vector3f lambdas_model, Vector3f lambdas_sliding, Vector3f kgains, Vector3f pgains, Vector3f sigma);

  void Log_CC0(Vector3f U, Vector3f dwm, Vector3f error, Vector3f d_error) const;
  
  void Log_CC1(Vector3f w_r, Vector3f d_wr, Vector3f w, Vector3f ah) const;
  
  void Log_CC2(float dxr_roll, float dxr_pitch, float dxr_yaw, 
               float ddxr_roll, float ddxr_pitch, float ddxr_yaw) const;

  void Log_CC3(Vector3f s_) const;

  // Constructor
  Custom_Att_Controller();

  // Destructor
  ~Custom_Att_Controller();

  // private data and function members
 private:
  // Block states
  DW_Custom_Att_Controller_T Block_State;

  Vector3f ys, wm, dotw_m;
  Vector3f a_hat, da_hat;
  Vector3f wr, dwr;
  Vector3f s;
  Vector3f derror;
  Matrix3f Y, P, Kd, Lm, Ls, SigmaM;
};
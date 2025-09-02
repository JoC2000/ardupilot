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
  void step(float x_d[3], float dx[3], float x[3], float U[3], float dt);

  // Unwrap function
  float unwrap_angle(float prev, float current);

  void Log_CC0(float u_roll, float u_pitch, float u_yaw, 
           float xm_r, float xm_p, float xm_y,
           float dxm_r, float dxm_p, float dxm_y,
           float ddxmr, float ddxmp, float ddxmy) const;
  
  void Log_CC1(float ah_r1, float ah_r2, float ah_p1, 
           float ah_p2, float ah_y1, float ah_y2, float roll_e, float pitch_e, float yaw_e) const;
  
  void Log_CC2(float dxr_roll, float dxr_pitch, float dxr_yaw, 
               float ddxr_roll, float ddxr_pitch, float ddxr_yaw) const;

  void Log_CC3(float dxr_roll, float dxr_pitch, float dxr_yaw, 
               float ddxr_roll, float ddxr_pitch, float ddxr_yaw) const;
  
  void Log_CC4(float s_roll_c, float s_pitch_c, float s_yaw_c,
               float s_roll_a, float s_pitch_a, float s_yaw_a) const;

  // Constructor
  Custom_Att_Controller();

  // Destructor
  ~Custom_Att_Controller();

  // private data and function members
 private:
  // Block states
  DW_Custom_Att_Controller_T Block_State;

  // Tuning parameters
  float l1,l2,l3,l4;

  float lambda_controller;
  
  float k2,k3,k4;

  float lambda_adaptation;

  float P1_gain,P1_11,P1_22;
  
  float P2_gain,P2_11,P2_22;

  float P3_gain,P3_11,P3_22;

  float sigma;

  float prev_yaw_ref,prev_yaw_real,prev_yaw_model;
};
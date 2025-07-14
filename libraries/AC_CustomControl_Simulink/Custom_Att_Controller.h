#pragma once
#include <cmath>

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

  // Constructor
  Custom_Att_Controller();

  // Destructor
  ~Custom_Att_Controller();

  // private data and function members
 private:
  // Block states
  DW_Custom_Att_Controller_T Block_State;
};
//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Custom_Att_Controller.cpp
//
// Code generated for Simulink model 'Custom_Att_Controller'.
//
// Model version                  : 1.1
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jun 25 10:47:02 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "Custom_Att_Controller.h"
#include "rtwtypes.h"

// Model step function
void Custom_Att_Controller::step(real32_T arg_x_d[3], real32_T arg_d_x[3],
  real32_T arg_x_real[3], real32_T arg_Out1[3])
{
  int32_T i;
  real32_T tmp[6];
  real32_T arg_Out1_tmp;
  real32_T ddxr;
  real32_T ddxr_0;
  real32_T ddxr_1;
  real32_T ddxr_idx_0;
  real32_T ddxr_idx_1;
  real32_T ddxr_idx_2;
  real32_T rtb_ddx_m_idx_0;
  real32_T rtb_ddx_m_idx_1;
  real32_T rtb_ddx_m_idx_2;
  real32_T s;
  real32_T s_0;
  real32_T s_1;
  real32_T s_idx_0;
  real32_T s_idx_1;
  real32_T s_idx_2_tmp;

  // MATLAB Function: '<Root>/Reference Model' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'
  //   Inport: '<Root>/x_reference'

  rtb_ddx_m_idx_0 = (200.0F * arg_x_d[0] - 50.0F *
                     Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0])
    - 600.0F * Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[0];
  rtb_ddx_m_idx_1 = (600.0F * arg_x_d[1] - 50.0F *
                     Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1])
    - 600.0F * Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[1];
  rtb_ddx_m_idx_2 = (50.0F * arg_x_d[2] - 15.0F *
                     Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2])
    - 50.0F * Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[2];

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'
  //   Inport: '<Root>/dx_measured'
  //   Inport: '<Root>/x_measured'
  //   MATLAB Function: '<Root>/Controller'

  ddxr = arg_d_x[0] - Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0];
  ddxr_idx_0 = rtb_ddx_m_idx_0 - 40.0F * ddxr;
  s = arg_x_real[0] - Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[0];
  s_idx_0 = arg_d_x[0] -
    (Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] - 40.0F * s);

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  ddxr_0 = rtb_ddx_m_idx_0 - 80.0F * ddxr;
  s_0 = arg_d_x[0] - (Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0]
                      - 80.0F * s);

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'
  //   Inport: '<Root>/dx_measured'
  //   Inport: '<Root>/x_measured'
  //   MATLAB Function: '<Root>/Controller'

  ddxr = arg_d_x[1] - Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1];
  ddxr_idx_1 = rtb_ddx_m_idx_1 - 40.0F * ddxr;
  s = arg_x_real[1] - Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[1];
  s_idx_1 = arg_d_x[1] -
    (Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] - 40.0F * s);

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  ddxr_1 = rtb_ddx_m_idx_1 - 80.0F * ddxr;
  s_1 = arg_d_x[1] - (Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1]
                      - 80.0F * s);

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'
  //   Inport: '<Root>/dx_measured'
  //   Inport: '<Root>/x_measured'
  //   MATLAB Function: '<Root>/Controller'

  ddxr = arg_d_x[2] - Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2];
  ddxr_idx_2 = rtb_ddx_m_idx_2 - 40.0F * ddxr;
  s = arg_x_real[2] - Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[2];
  s_idx_2_tmp = arg_d_x[2] -
    (Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2] - 40.0F * s);

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  arg_Out1_tmp = arg_d_x[1] * arg_d_x[2];

  // Outport: '<Root>/u_out' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  //   MATLAB Function: '<Root>/Controller'

  arg_Out1[0] = (arg_Out1_tmp *
                 Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[1] +
                 ddxr_0 *
                 Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[0]) +
    -7.5F * s_0;

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  ddxr_0 = arg_d_x[0] * arg_d_x[2];

  // Outport: '<Root>/u_out' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  //   MATLAB Function: '<Root>/Controller'

  arg_Out1[1] = (ddxr_0 *
                 Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[3] +
                 ddxr_1 *
                 Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[2]) +
    -6.4F * s_1;

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  ddxr_1 = arg_d_x[0] * arg_d_x[1];

  // Outport: '<Root>/u_out' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'
  //   MATLAB Function: '<Root>/Controller'

  arg_Out1[2] = (arg_d_x[2] -
                 (Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2] -
                  80.0F * s)) * -2.5F + ((rtb_ddx_m_idx_2 - 80.0F * ddxr) *
    Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[4] + ddxr_1 *
    Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[5]);

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator'

  ddxr = -s_idx_0 * 0.0F;
  tmp[0] = (ddxr * arg_Out1_tmp + -s_idx_0 * ddxr_idx_0) *
    Custom_Att_Controller_P.DiscreteTimeIntegrator_gainval;
  s = -s_idx_1 * 0.0F;
  tmp[2] = (s * ddxr_0 + -s_idx_1 * ddxr_idx_1) *
    Custom_Att_Controller_P.DiscreteTimeIntegrator_gainval;
  s_0 = -s_idx_2_tmp * 0.0F;
  tmp[4] = (s_0 * ddxr_1 + -s_idx_2_tmp * ddxr_idx_2) *
    Custom_Att_Controller_P.DiscreteTimeIntegrator_gainval;
  tmp[1] = (ddxr * ddxr_idx_0 + -s_idx_0 * arg_Out1_tmp) *
    Custom_Att_Controller_P.DiscreteTimeIntegrator_gainval;
  tmp[3] = (s * ddxr_idx_1 + -s_idx_1 * ddxr_0) *
    Custom_Att_Controller_P.DiscreteTimeIntegrator_gainval;
  tmp[5] = (s_0 * ddxr_idx_2 + -s_idx_2_tmp * ddxr_1) *
    Custom_Att_Controller_P.DiscreteTimeIntegrator_gainval;

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
  for (i = 0; i < 6; i++) {
    Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[i] += tmp[i];
  }

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'

  Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[0] +=
    Custom_Att_Controller_P.DiscreteTimeIntegrator2_gainval *
    Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0];

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'

  Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] +=
    Custom_Att_Controller_P.DiscreteTimeIntegrator1_gainval * rtb_ddx_m_idx_0;

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'

  Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[1] +=
    Custom_Att_Controller_P.DiscreteTimeIntegrator2_gainval *
    Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1];

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'

  Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] +=
    Custom_Att_Controller_P.DiscreteTimeIntegrator1_gainval * rtb_ddx_m_idx_1;

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'

  Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[2] +=
    Custom_Att_Controller_P.DiscreteTimeIntegrator2_gainval *
    Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2];

  // Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' incorporates:
  //   DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'

  Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2] +=
    Custom_Att_Controller_P.DiscreteTimeIntegrator1_gainval * rtb_ddx_m_idx_2;
}

// Model initialize function
void Custom_Att_Controller::initialize()
{
  {
    int32_T i;

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' 
    for (i = 0; i < 6; i++) {
      Custom_Att_Controller_DW.DiscreteTimeIntegrator_DSTATE[i] =
        Custom_Att_Controller_P.DiscreteTimeIntegrator_IC;
    }

    // End of InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' 

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' 
    Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[0] =
      Custom_Att_Controller_P.DiscreteTimeIntegrator2_IC;

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' 
    Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[0] =
      Custom_Att_Controller_P.DiscreteTimeIntegrator1_IC;

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' 
    Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[1] =
      Custom_Att_Controller_P.DiscreteTimeIntegrator2_IC;

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' 
    Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[1] =
      Custom_Att_Controller_P.DiscreteTimeIntegrator1_IC;

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' 
    Custom_Att_Controller_DW.DiscreteTimeIntegrator2_DSTATE[2] =
      Custom_Att_Controller_P.DiscreteTimeIntegrator2_IC;

    // InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' 
    Custom_Att_Controller_DW.DiscreteTimeIntegrator1_DSTATE[2] =
      Custom_Att_Controller_P.DiscreteTimeIntegrator1_IC;
  }
}

// Model terminate function
void Custom_Att_Controller::terminate()
{
  // (no terminate code required)
}

const char_T* Custom_Att_Controller::RT_MODEL_Custom_Att_Controlle_T::
  getErrorStatus() const
{
  return (errorStatus);
}

void Custom_Att_Controller::RT_MODEL_Custom_Att_Controlle_T::setErrorStatus(
  const char_T* const volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() :
  Custom_Att_Controller_DW(),
  Custom_Att_Controller_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
Custom_Att_Controller::~Custom_Att_Controller() = default;

// Real-Time Model get method
Custom_Att_Controller::RT_MODEL_Custom_Att_Controlle_T * Custom_Att_Controller::
  getRTM()
{
  return (&Custom_Att_Controller_M);
}

//
// File trailer for generated code.
//
// [EOF]
//

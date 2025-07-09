//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Custom_Att_Controller.cpp
//
// Code generated for Simulink model 'Custom_Att_Controller'.
//
// Model version                  : 1.49
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Jul  8 17:45:19 2025
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
  int32_T i_0;
  real32_T dxm[6];
  real32_T tmp[6];
  real32_T arg_Out1_tmp;
  real32_T ddxr_tmp_idx_0;
  real32_T ddxr_tmp_idx_1;
  real32_T ddxr_tmp_idx_2;
  real32_T s;
  real32_T s_0;
  real32_T s_1;
  real32_T s_idx_0;
  real32_T s_idx_1;
  real32_T s_idx_2;

  real32_T lambda_controller = 1.87;
  real32_T k2 = 0.112;
  real32_T k3 = 0.112;
  real32_T k4 = 0.110;

  real32_T lambda_adaptation = 0.065;
  real32_T P1_gain = 0.0012;
  real32_T P1_11 = 0.75;
  real32_T P1_22 = 0.15;

  real32_T P2_gain = 0.0012;
  real32_T P2_11 = 0.75;
  real32_T P2_22 = 0.15;

  real32_T P3_gain = 0.0008;
  real32_T P3_11 = 0.08;
  real32_T P3_22 = 0.10;

  real32_T sigma = 0.025;

  static const real32_T c[18]{ 0.0F, 19.662F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 19.662F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 3.478F };

  static const real32_T b[36]{ 0.0F, -19.662F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    -9.13F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -19.662F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, -9.13F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -3.478F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, -5.48F };

  // DiscreteIntegrator: '<Root>/x_m' incorporates:
  //   Inport: '<Root>/x_measured'

  if (Custom_Att_Controller_DW.x_m_IC_LOADING != 0) {
    Custom_Att_Controller_DW.x_m_DSTATE[0] = arg_x_real[0];
    Custom_Att_Controller_DW.x_m_DSTATE[1] = arg_x_real[1];
    Custom_Att_Controller_DW.x_m_DSTATE[2] = arg_x_real[2];
  }

  // DiscreteIntegrator: '<Root>/dx_m_integrator' incorporates:
  //   Inport: '<Root>/dx_measured'

  if (Custom_Att_Controller_DW.dx_m_integrator_IC_LOADING != 0) {
    Custom_Att_Controller_DW.dx_m_integrator_DSTATE[0] = arg_d_x[0];
    Custom_Att_Controller_DW.dx_m_integrator_DSTATE[1] = arg_d_x[1];
    Custom_Att_Controller_DW.dx_m_integrator_DSTATE[2] = arg_d_x[2];
  }

  // SignalConversion generated from: '<S3>/ SFunction ' incorporates:
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   DiscreteIntegrator: '<Root>/x_m'
  //   MATLAB Function: '<Root>/Reference Model'

  tmp[0] = Custom_Att_Controller_DW.x_m_DSTATE[0];
  tmp[1] = Custom_Att_Controller_DW.dx_m_integrator_DSTATE[0];
  tmp[2] = Custom_Att_Controller_DW.x_m_DSTATE[1];
  tmp[3] = Custom_Att_Controller_DW.dx_m_integrator_DSTATE[1];
  tmp[4] = Custom_Att_Controller_DW.x_m_DSTATE[2];
  tmp[5] = Custom_Att_Controller_DW.dx_m_integrator_DSTATE[2];

  // MATLAB Function: '<Root>/Reference Model' incorporates:
  //   Inport: '<Root>/x_reference'

  for (i = 0; i < 6; i++) {
    s_idx_2 = 0.0F;
    for (i_0 = 0; i_0 < 6; i_0++) {
      s_idx_2 += b[6 * i_0 + i] * tmp[i_0];
    }

    dxm[i] = ((c[i + 6] * arg_x_d[1] + c[i] * arg_x_d[0]) + c[i + 12] * arg_x_d
              [2]) + s_idx_2;
  }

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   DiscreteIntegrator: '<Root>/x_m'
  //   Inport: '<Root>/dx_measured'
  //   Inport: '<Root>/x_measured'
  //   MATLAB Function: '<Root>/Controller'

  ddxr_tmp_idx_0 = arg_d_x[0] - Custom_Att_Controller_DW.dx_m_integrator_DSTATE
    [0];
  s = arg_x_real[0] - Custom_Att_Controller_DW.x_m_DSTATE[0];
  s_idx_0 = arg_d_x[0] - (Custom_Att_Controller_DW.dx_m_integrator_DSTATE[0] -
    lambda_adaptation * s);

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  s_0 = arg_d_x[0] - (Custom_Att_Controller_DW.dx_m_integrator_DSTATE[0] - lambda_controller
                      * s);

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   DiscreteIntegrator: '<Root>/x_m'
  //   Inport: '<Root>/dx_measured'
  //   Inport: '<Root>/x_measured'
  //   MATLAB Function: '<Root>/Controller'

  ddxr_tmp_idx_1 = arg_d_x[1] - Custom_Att_Controller_DW.dx_m_integrator_DSTATE
    [1];
  s = arg_x_real[1] - Custom_Att_Controller_DW.x_m_DSTATE[1];
  s_idx_1 = arg_d_x[1] - (Custom_Att_Controller_DW.dx_m_integrator_DSTATE[1] -
    lambda_adaptation * s);

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  s_1 = arg_d_x[1] - (Custom_Att_Controller_DW.dx_m_integrator_DSTATE[1] - lambda_controller
                      * s);

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   DiscreteIntegrator: '<Root>/x_m'
  //   Inport: '<Root>/dx_measured'
  //   Inport: '<Root>/x_measured'
  //   MATLAB Function: '<Root>/Controller'

  ddxr_tmp_idx_2 = arg_d_x[2] - Custom_Att_Controller_DW.dx_m_integrator_DSTATE
    [2];
  s = arg_x_real[2] - Custom_Att_Controller_DW.x_m_DSTATE[2];
  s_idx_2 = arg_d_x[2] - (Custom_Att_Controller_DW.dx_m_integrator_DSTATE[2] -
    lambda_adaptation * s);

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  arg_Out1_tmp = arg_d_x[1] * arg_d_x[2];

  // Outport: '<Root>/u_out' incorporates:
  //   DiscreteIntegrator: '<Root>/ah'
  //   MATLAB Function: '<Root>/Controller'
  //   MATLAB Function: '<Root>/Reference Model'

  arg_Out1[0] = ((dxm[1] - lambda_controller * ddxr_tmp_idx_0) *
                 Custom_Att_Controller_DW.ah_DSTATE[0] + arg_Out1_tmp *
                 Custom_Att_Controller_DW.ah_DSTATE[1]) + -k2 * s_0;

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  s_0 = arg_d_x[0] * arg_d_x[2];

  // Outport: '<Root>/u_out' incorporates:
  //   DiscreteIntegrator: '<Root>/ah'
  //   MATLAB Function: '<Root>/Controller'
  //   MATLAB Function: '<Root>/Reference Model'

  arg_Out1[1] = ((dxm[3] - lambda_controller * ddxr_tmp_idx_1) *
                 Custom_Att_Controller_DW.ah_DSTATE[2] + s_0 *
                 Custom_Att_Controller_DW.ah_DSTATE[3]) + -k3 * s_1;

  // MATLAB Function: '<Root>/Controller' incorporates:
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'

  s_1 = arg_d_x[0] * arg_d_x[1];

  // Outport: '<Root>/u_out' incorporates:
  //   DiscreteIntegrator: '<Root>/ah'
  //   DiscreteIntegrator: '<Root>/dx_m_integrator'
  //   Inport: '<Root>/dx_measured'
  //   MATLAB Function: '<Root>/Adaptation Law'
  //   MATLAB Function: '<Root>/Controller'
  //   MATLAB Function: '<Root>/Reference Model'

  arg_Out1[2] = (arg_d_x[2] - (Custom_Att_Controller_DW.dx_m_integrator_DSTATE[2]
    - lambda_controller * s)) * -k4 + ((dxm[5] - lambda_controller * ddxr_tmp_idx_2) *
    Custom_Att_Controller_DW.ah_DSTATE[4] + s_1 *
    Custom_Att_Controller_DW.ah_DSTATE[5]);

  // MATLAB Function: '<Root>/Adaptation Law' incorporates:
  //   DiscreteIntegrator: '<Root>/ah'
  //   MATLAB Function: '<Root>/Reference Model'
  //
  ddxr_tmp_idx_0 = (dxm[1] - lambda_adaptation * ddxr_tmp_idx_0) * s_idx_0;
  s_idx_0 *= arg_Out1_tmp;
  ddxr_tmp_idx_1 = (dxm[3] - lambda_adaptation * ddxr_tmp_idx_1) * s_idx_1;
  s_idx_1 *= s_0;
  ddxr_tmp_idx_2 = (dxm[5] - lambda_adaptation * ddxr_tmp_idx_2) * s_idx_2;
  s_idx_2 *= s_1;
  tmp[0] = ((-(P1_gain*P1_11) * ddxr_tmp_idx_0 + -0.0F * s_idx_0) - ((P1_gain*P1_11*sigma) *
             Custom_Att_Controller_DW.ah_DSTATE[0] +
             Custom_Att_Controller_DW.ah_DSTATE[1] * 0.0F)) *
    Custom_Att_Controller_P.ah_gainval;
  tmp[2] = ((-(P2_gain*P2_11) * ddxr_tmp_idx_1 + -0.0F * s_idx_1) - ((P2_gain*P2_11*sigma) *
             Custom_Att_Controller_DW.ah_DSTATE[2] + 0.0F *
             Custom_Att_Controller_DW.ah_DSTATE[3])) *
    Custom_Att_Controller_P.ah_gainval;
  tmp[4] = ((-(P3_gain*P3_11) * ddxr_tmp_idx_2 + -0.0F * s_idx_2) - ((P3_gain*P3_11*sigma) *
             Custom_Att_Controller_DW.ah_DSTATE[4] + 0.0F *
             Custom_Att_Controller_DW.ah_DSTATE[5])) *
    Custom_Att_Controller_P.ah_gainval;
  tmp[1] = ((-0.0F * ddxr_tmp_idx_0 + -(P1_gain*P1_22) * s_idx_0) -
            (Custom_Att_Controller_DW.ah_DSTATE[0] * 0.0F +
             Custom_Att_Controller_DW.ah_DSTATE[1] * (P1_gain*P1_22*sigma))) *
    Custom_Att_Controller_P.ah_gainval;
  tmp[3] = ((-0.0F * ddxr_tmp_idx_1 + -(P2_gain*P2_22) * s_idx_1) - (0.0F *
             Custom_Att_Controller_DW.ah_DSTATE[2] + (P2_gain*P2_22*sigma) *
             Custom_Att_Controller_DW.ah_DSTATE[3])) *
    Custom_Att_Controller_P.ah_gainval;
  tmp[5] = ((-0.0F * ddxr_tmp_idx_2 + -(P3_gain*P3_22) * s_idx_2) - (0.0F *
             Custom_Att_Controller_DW.ah_DSTATE[4] + (P3_gain*P3_22*sigma) *
             Custom_Att_Controller_DW.ah_DSTATE[5])) *
    Custom_Att_Controller_P.ah_gainval;

  // Update for DiscreteIntegrator: '<Root>/ah'
  for (i = 0; i < 6; i++) {
    Custom_Att_Controller_DW.ah_DSTATE[i] += tmp[i];
  }

  // Update for DiscreteIntegrator: '<Root>/x_m' incorporates:
  //   MATLAB Function: '<Root>/Reference Model'

  Custom_Att_Controller_DW.x_m_IC_LOADING = 0U;
  s_idx_2 = Custom_Att_Controller_DW.x_m_DSTATE[0];
  ddxr_tmp_idx_2 = Custom_Att_Controller_DW.x_m_DSTATE[1];
  s_idx_1 = Custom_Att_Controller_DW.x_m_DSTATE[2];
  Custom_Att_Controller_DW.x_m_DSTATE[0] = Custom_Att_Controller_P.x_m_gainval *
    dxm[0] + s_idx_2;
  Custom_Att_Controller_DW.x_m_DSTATE[1] = Custom_Att_Controller_P.x_m_gainval *
    dxm[2] + ddxr_tmp_idx_2;
  Custom_Att_Controller_DW.x_m_DSTATE[2] = Custom_Att_Controller_P.x_m_gainval *
    dxm[4] + s_idx_1;

  // Update for DiscreteIntegrator: '<Root>/dx_m_integrator' incorporates:
  //   MATLAB Function: '<Root>/Reference Model'

  Custom_Att_Controller_DW.dx_m_integrator_IC_LOADING = 0U;
  s_idx_2 = Custom_Att_Controller_DW.dx_m_integrator_DSTATE[0];
  ddxr_tmp_idx_2 = Custom_Att_Controller_DW.dx_m_integrator_DSTATE[1];
  s_idx_1 = Custom_Att_Controller_DW.dx_m_integrator_DSTATE[2];
  Custom_Att_Controller_DW.dx_m_integrator_DSTATE[0] =
    Custom_Att_Controller_P.dx_m_integrator_gainval * dxm[1] + s_idx_2;
  Custom_Att_Controller_DW.dx_m_integrator_DSTATE[1] =
    Custom_Att_Controller_P.dx_m_integrator_gainval * dxm[3] + ddxr_tmp_idx_2;
  Custom_Att_Controller_DW.dx_m_integrator_DSTATE[2] =
    Custom_Att_Controller_P.dx_m_integrator_gainval * dxm[5] + s_idx_1;
}

// Model initialize function
void Custom_Att_Controller::initialize()
{
  {
    int32_T i;

    // InitializeConditions for DiscreteIntegrator: '<Root>/ah'
    for (i = 0; i < 6; i++) {
      Custom_Att_Controller_DW.ah_DSTATE[i] = Custom_Att_Controller_P.ah_IC;
    }

    // End of InitializeConditions for DiscreteIntegrator: '<Root>/ah'

    // InitializeConditions for DiscreteIntegrator: '<Root>/x_m'
    Custom_Att_Controller_DW.x_m_IC_LOADING = 1U;

    // InitializeConditions for DiscreteIntegrator: '<Root>/dx_m_integrator'
    Custom_Att_Controller_DW.dx_m_integrator_IC_LOADING = 1U;
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

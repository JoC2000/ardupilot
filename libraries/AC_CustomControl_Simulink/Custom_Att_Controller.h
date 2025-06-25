//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Custom_Att_Controller.h
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
#ifndef Custom_Att_Controller_h_
#define Custom_Att_Controller_h_
#include <cmath>
#include "rtwtypes.h"
#include "Custom_Att_Controller_types.h"

// Class declaration for model Custom_Att_Controller
class Custom_Att_Controller final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_Custom_Att_Controller_T {
    real32_T DiscreteTimeIntegrator_DSTATE[6];// '<Root>/Discrete-Time Integrator' 
    real32_T DiscreteTimeIntegrator2_DSTATE[3];// '<Root>/Discrete-Time Integrator2' 
    real32_T DiscreteTimeIntegrator1_DSTATE[3];// '<Root>/Discrete-Time Integrator1' 
  };

  // Parameters (default storage)
  struct P_Custom_Att_Controller_T {
    real32_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<Root>/Discrete-Time Integrator'

    real32_T DiscreteTimeIntegrator_IC;
                                // Computed Parameter: DiscreteTimeIntegrator_IC
                                   //  Referenced by: '<Root>/Discrete-Time Integrator'

    real32_T DiscreteTimeIntegrator2_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator2_gainval
                             //  Referenced by: '<Root>/Discrete-Time Integrator2'

    real32_T DiscreteTimeIntegrator2_IC;
                               // Computed Parameter: DiscreteTimeIntegrator2_IC
                                  //  Referenced by: '<Root>/Discrete-Time Integrator2'

    real32_T DiscreteTimeIntegrator1_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainval
                             //  Referenced by: '<Root>/Discrete-Time Integrator1'

    real32_T DiscreteTimeIntegrator1_IC;
                               // Computed Parameter: DiscreteTimeIntegrator1_IC
                                  //  Referenced by: '<Root>/Discrete-Time Integrator1'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_Custom_Att_Controlle_T {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  Custom_Att_Controller(Custom_Att_Controller const&) = delete;

  // Assignment Operator
  Custom_Att_Controller& operator= (Custom_Att_Controller const&) & = delete;

  // Move Constructor
  Custom_Att_Controller(Custom_Att_Controller &&) = delete;

  // Move Assignment Operator
  Custom_Att_Controller& operator= (Custom_Att_Controller &&) = delete;

  // Real-Time Model get method
  Custom_Att_Controller::RT_MODEL_Custom_Att_Controlle_T * getRTM();

  // Tunable parameters
  static P_Custom_Att_Controller_T Custom_Att_Controller_P;

  // model initialize function
  void initialize();

  // model step function
  void step(real32_T arg_x_d[3], real32_T arg_d_x[3], real32_T arg_x_real[3],
            real32_T arg_Out1[3]);

  // model terminate function
  static void terminate();

  // Constructor
  Custom_Att_Controller();

  // Destructor
  ~Custom_Att_Controller();

  // private data and function members
 private:
  // Block states
  DW_Custom_Att_Controller_T Custom_Att_Controller_DW;

  // Real-Time Model
  RT_MODEL_Custom_Att_Controlle_T Custom_Att_Controller_M;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'Custom_Att_Controller'
//  '<S1>'   : 'Custom_Att_Controller/Adaptation Law'
//  '<S2>'   : 'Custom_Att_Controller/Controller'
//  '<S3>'   : 'Custom_Att_Controller/Reference Model'

#endif                                 // Custom_Att_Controller_h_

//
// File trailer for generated code.
//
// [EOF]
//

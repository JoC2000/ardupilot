#include "Custom_Att_Controller.h"

float Custom_Att_Controller::unwrap_angle(float prev, float current)
{
  float diff = current - prev;
  if (diff > M_PI) {
    current -= 2.0f * M_PI;
  } else if (diff < -M_PI) {
    current += 2.0f * M_PI;
  }
  return current;
}

void Custom_Att_Controller::step(float x_d[3], float dx[3], float x[3], float U[3], float dt)
{
  int i;
  int i_0;
  float xm[6];
  float dxm[6];
  float ah[6];
  float error[3];
  float derror[3];
  float dxr_adaptation[3];
  float dxr_controller[3];
  float ddxr_adaptation[3];
  float ddxr_controller[3];
  float s_adaptation[3];
  float s_controller[3];
  float temp;

  // Tuning parameters
  float lambda_controller = 2.78F;
  float k2 = 0.387F;
  float k3 = 0.387F;
  float k4 = 0.235F;

  float lambda_adaptation = 0.1F;
  float P1_gain = 0.012F;
  float P1_11 = 0.75F;
  float P1_22 = 0.15F;

  float P2_gain = 0.012F;
  float P2_11 = 0.75F;
  float P2_22 = 0.15F;

  float P3_gain = 0.012F;
  float P3_11 = 0.25F;
  float P3_22 = 0.10F;

  float sigma = 0.025F;
  
  // Unwrap angles to avoid discontinuities
  static float prev_yaw_ref = 0.0F;
  static float prev_yaw_real = 0.0F;
  static float prev_yaw_model = 0.0F;

  float raw_yaw_ref = x_d[2];
  float raw_yaw_real = x[2];
  float raw_yaw_model = Block_State.x_m[2];

  float unwrapped_yaw_ref = unwrap_angle(prev_yaw_ref, raw_yaw_ref);
  float unwrapped_yaw_real = unwrap_angle(prev_yaw_real, raw_yaw_real);
  float unwrapped_yaw_model = unwrap_angle(prev_yaw_model, raw_yaw_model);  
  
  prev_yaw_ref = unwrapped_yaw_ref;
  prev_yaw_real = unwrapped_yaw_real;
  prev_yaw_model = unwrapped_yaw_model;

  x_d[2] = unwrapped_yaw_ref;
  x[2] = unwrapped_yaw_real;
  Block_State.x_m[2] = unwrapped_yaw_model;

  static const float c[18]{ 0.0F, 19.662F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 19.662F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 3.478F };

  static const float b[36]{ 0.0F, -19.662F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    -9.13F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -19.662F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, -9.13F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -3.478F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, -5.48F };

  // x_m discrete integrator initial values

  if (Block_State.x_m_IC_LOADING != 0) {
    Block_State.x_m[0] = x[0];
    Block_State.x_m[1] = x[1];
    Block_State.x_m[2] = x[2];
  }

  // dx_m discrete integrator initial values

  if (Block_State.dx_m_integrator_IC_LOADING != 0) {
    Block_State.dx_m[0] = dx[0];
    Block_State.dx_m[1] = dx[1];
    Block_State.dx_m[2] = dx[2];
  }

  // Model Reference
  xm[0] = Block_State.x_m[0];
  xm[1] = Block_State.dx_m[0];
  xm[2] = Block_State.x_m[1];
  xm[3] = Block_State.dx_m[1];
  xm[4] = Block_State.x_m[2];
  xm[5] = Block_State.dx_m[2];

  if (fabs(x[2] - Block_State.x_m[2]) > 0.005F) {
    Block_State.x_m[0] = 0.0F;
    Block_State.x_m[1] = 0.0F;
  }

  for (i = 0; i < 6; i++) {
    temp = 0.0F;
    for (i_0 = 0; i_0 < 6; i_0++) {
      temp += b[6 * i_0 + i] * xm[i_0];
    }

    dxm[i] = ((c[i + 6] * x_d[1] + c[i] * x_d[0]) + c[i + 12] * x_d[2]) + temp;
  }
  /*
  Model Reference outputs in State Space form:
  
  dx_m = A*xm + B*x_d

  where:
  dx_m = [roll,
          droll, 
          pitch, 
          dpitch,
          yaw,
          dyaw]
  */

  // Error definitions
  error[0] = x[0] - Block_State.x_m[0];
  derror[0] = dx[0] - Block_State.dx_m[0];

  error[1] = x[1] - Block_State.x_m[1];
  derror[1] = dx[1] - Block_State.dx_m[1];

  error[2] = x[2] - Block_State.x_m[2];
  derror[2] = dx[2] - Block_State.dx_m[2];
  
  // Controller
  dxr_controller[0] = Block_State.dx_m[0] - lambda_controller * error[0];
  s_controller[0] = dx[0] - dxr_controller[0];
  ddxr_controller[0] = dxm[1] - lambda_controller * derror[0];

  dxr_controller[1] = Block_State.dx_m[1] - lambda_controller * error[1];
  s_controller[1] = dx[1] - dxr_controller[1];
  ddxr_controller[1] = dxm[3] - lambda_controller * derror[1];

  dxr_controller[2] = Block_State.dx_m[2] - lambda_controller * error[2];
  s_controller[2] = dx[2] - dxr_controller[2];
  ddxr_controller[2] = dxm[5] - lambda_controller * derror[2];

  // Controller Outputs
  U[0] = -k2 * s_controller[0] + (ddxr_controller[0] * Block_State.ah[0] + dx[1] * dx[2] * Block_State.ah[1]);  

  U[1] = -k3 * s_controller[1] + (ddxr_controller[1] * Block_State.ah[2] + dx[0] * dx[2] * Block_State.ah[3]);

  U[2] = -k4 * s_controller[2] + (ddxr_controller[2] * Block_State.ah[4] + dx[0] * dx[1] * Block_State.ah[5]);

  // Adaptation Law
  dxr_adaptation[0] = Block_State.dx_m[0] - lambda_adaptation * error[0];
  s_adaptation[0] = dx[0] - dxr_adaptation[0];
  ddxr_adaptation[0] = (dxm[1] - lambda_adaptation * derror[0]);

  dxr_adaptation[1] = Block_State.dx_m[1] - lambda_adaptation * error[1];
  s_adaptation[1] = dx[1] - dxr_adaptation[1];
  ddxr_adaptation[1] = (dxm[3] - lambda_adaptation * derror[1]);
  
  dxr_adaptation[2] = Block_State.dx_m[2] - lambda_adaptation * error[2];
  s_adaptation[2] = dx[2] - dxr_adaptation[2];
  ddxr_adaptation[2] = (dxm[5] - lambda_adaptation * derror[2]);

  ah[0] = ((-(P1_gain*P1_11) * ddxr_adaptation[0] * s_adaptation[0] + -0.0F * s_adaptation[0] * dx[1] * dx[2]) - ((P1_gain*P1_11*sigma) * Block_State.ah[0] + Block_State.ah[1] * 0.0F)) * dt;

  ah[2] = ((-(P2_gain*P2_11) * ddxr_adaptation[1] * s_adaptation[1] + -0.0F * s_adaptation[1] * dx[0] * dx[2]) - ((P2_gain*P2_11*sigma) * Block_State.ah[2] + 0.0F * Block_State.ah[3])) * dt;

  ah[4] = ((-(P3_gain*P3_11) * ddxr_adaptation[2] * s_adaptation[2] + -0.0F * s_adaptation[2] * dx[0] * dx[1]) - ((P3_gain*P3_11*sigma) * Block_State.ah[4] + 0.0F * Block_State.ah[5])) * dt;
  
  ah[1] = ((-0.0F * ddxr_adaptation[0] * s_adaptation[0] + -(P1_gain*P1_22) * s_adaptation[0] * dx[1] * dx[2]) - (Block_State.ah[0] * 0.0F + Block_State.ah[1] * (P1_gain*P1_22*sigma))) * dt;

  ah[3] = ((-0.0F * ddxr_adaptation[1] * s_adaptation[1] + -(P2_gain*P2_22) * s_adaptation[1] * dx[0] * dx[2]) - (0.0F * Block_State.ah[2] + (P2_gain*P2_22*sigma) * Block_State.ah[3])) * dt;

  ah[5] = ((-0.0F * ddxr_adaptation[2] * s_adaptation[2] + -(P3_gain*P3_22) * s_adaptation[2] * dx[0] * dx[1]) - (0.0F * Block_State.ah[4] + (P3_gain*P3_22*sigma) * Block_State.ah[5])) * dt;

  // Update for ah discrete integrator
  for (i = 0; i < 6; i++) {
    Block_State.ah[i] += ah[i];
  }

  // Update for x_m dicrete integrator

  Block_State.x_m_IC_LOADING = 0U;
  Block_State.x_m[0] = dt * dxm[0] + Block_State.x_m[0];
  Block_State.x_m[1] = dt * dxm[2] + Block_State.x_m[1];
  Block_State.x_m[2] = dt * dxm[4] + Block_State.x_m[2];

  // Update for dx_m discrete integrator

  Block_State.dx_m_integrator_IC_LOADING = 0U;
  Block_State.dx_m[0] = dt * dxm[1] + Block_State.dx_m[0];
  Block_State.dx_m[1] = dt * dxm[3] + Block_State.dx_m[1];
  Block_State.dx_m[2] = dt * dxm[5] + Block_State.dx_m[2];
}

void Custom_Att_Controller::initialize()
{
  {
    int i;

    // Initialize Conditions for ah integrator
    for (i = 0; i < 6; i++) {
      Block_State.ah[i] = 0U;
    }

    // InitializeConditions for x_m integrator
    Block_State.x_m_IC_LOADING = 1U;

    // InitializeConditions for dx_m integrator
    Block_State.dx_m_integrator_IC_LOADING = 1U;
  }
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() :
  Block_State()
{

}

Custom_Att_Controller::~Custom_Att_Controller() = default;
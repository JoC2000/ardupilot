#include "Custom_Att_Controller.h"

float Custom_Att_Controller::unwrap_angle(float prev_unwrapped, float current)
{
  float diff = current - fmod(prev_unwrapped, 2.0f * M_PI);

    // Bring diff into [-π, π]
    while (diff > M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;

    // Accumulate
    return prev_unwrapped + diff;
}

void Custom_Att_Controller::Log_CC0(float u_roll, float u_pitch, float u_yaw, float xm_r, float xm_p, float xm_y, float dxm_r, float dxm_p, float dxm_y, float ddxmr, float ddxmp, float ddxmy) const
{
  struct log_CC0 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC0_MSG),
    time_us     : AP_HAL::micros64(),
    u_roll      : degrees(u_roll),
    u_pitch     : degrees(u_pitch),
    u_yaw       : degrees(u_yaw),
    xm_roll     : degrees(xm_r),
    xm_pitch    : degrees(xm_p),
    xm_yaw      : wrap_360(degrees(xm_y)),
    dxm_roll    : degrees(dxm_r),
    dxm_pitch   : degrees(dxm_p),
    dxm_yaw     : degrees(dxm_y),
    ddxm_roll   : degrees(ddxmr),
    ddxm_pitch  : degrees(ddxmp),
    ddxm_yaw    : degrees(ddxmy),
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC1(float ah_r1, float ah_r2, float ah_p1, float ah_p2, float ah_y1, float ah_y2, float roll_e, float pitch_e, float yaw_e) const
{
  struct log_CC1 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC1_MSG),
    time_us     : AP_HAL::micros64(),
    ah_r1       : ah_r1,
    ah_r2       : ah_r2,
    ah_p1       : ah_p1,
    ah_p2       : ah_p2,
    ah_y1       : ah_y1,
    ah_y2       : ah_y2,
    roll_e      : degrees(roll_e),
    pitch_e     : degrees(pitch_e),
    yaw_e       : degrees(yaw_e),
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC2(float dxr_roll, float dxr_pitch, float dxr_yaw, float ddxr_roll, float ddxr_pitch, float ddxr_yaw) const
{
  struct log_CC2 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC2_MSG),
    time_us     : AP_HAL::micros64(),
    dxr_roll_c  : degrees(dxr_roll),
    dxr_pitch_c : degrees(dxr_pitch),
    dxr_yaw_c   : degrees(dxr_yaw),
    ddxr_roll_c : degrees(ddxr_roll),
    ddxr_pitch_c: degrees(ddxr_pitch),
    ddxr_yaw_c  : degrees(ddxr_yaw)
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC3(float dxr_roll, float dxr_pitch, float dxr_yaw, float ddxr_roll, float ddxr_pitch, float ddxr_yaw) const
{
  struct log_CC3 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC3_MSG),
    time_us     : AP_HAL::micros64(),
    dxr_roll_a  : degrees(dxr_roll),
    dxr_pitch_a : degrees(dxr_pitch),
    dxr_yaw_a   : degrees(dxr_yaw),
    ddxr_roll_a : degrees(ddxr_roll),
    ddxr_pitch_a: degrees(ddxr_pitch),
    ddxr_yaw_a  : degrees(ddxr_yaw),
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC4(float s_roll_c, float s_pitch_c, float s_yaw_c, float s_roll_a, float s_pitch_a, float s_yaw_a) const
{
  struct log_CC4 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC4_MSG),
    time_us     : AP_HAL::micros64(),
    s_roll_c    : degrees(s_roll_c),
    s_pitch_c   : degrees(s_pitch_c),
    s_yaw_c     : degrees(s_yaw_c),
    s_roll_a    : degrees(s_roll_a),
    s_pitch_a   : degrees(s_pitch_a),
    s_yaw_a     : degrees(s_yaw_a),
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

uint8_t log_div = 0;

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

  // Unwrap angles to avoid discontinuities
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

  static const float b[18]{ 0.0F, l1*l2, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, l1*l2, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, l3*l4 };

  static const float a[36]{ 0.0F, -(l1*l2), 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    -(l1+l2), 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -(l1*l2), 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, -(l1+l2), 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -(l3*l4), 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, -(l3+l4) };

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

  for (i = 0; i < 6; i++) {
    temp = 0.0F;
    for (i_0 = 0; i_0 < 6; i_0++) {
      temp += a[6 * i_0 + i] * xm[i_0];
    }

    dxm[i] = ((b[i + 6] * x_d[1] + b[i] * x_d[0]) + b[i + 12] * x_d[2]) + temp;
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

  U[1] = -k2 * s_controller[1] + (ddxr_controller[1] * Block_State.ah[2] + dx[0] * dx[2] * Block_State.ah[3]);

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

  ah[0] = ((-(P1_gain*P1_11) * ddxr_adaptation[0] * s_adaptation[0] + -0.0F * s_adaptation[0] * dx[1] * dx[2]) - (sigma * Block_State.ah[0] + 0.0F * Block_State.ah[1])) * dt;
  ah[1] = ((-0.0F * ddxr_adaptation[0] * s_adaptation[0] + -(P1_gain*P1_22) * s_adaptation[0] * dx[1] * dx[2]) - (0.0F * Block_State.ah[0] + sigma * Block_State.ah[1])) * dt;

  ah[2] = ((-(P2_gain*P2_11) * ddxr_adaptation[1] * s_adaptation[1] + -0.0F * s_adaptation[1] * dx[0] * dx[2]) - (sigma * Block_State.ah[2] + 0.0F * Block_State.ah[3])) * dt;
  ah[3] = ((-0.0F * ddxr_adaptation[1] * s_adaptation[1] + -(P2_gain*P2_22) * s_adaptation[1] * dx[0] * dx[2]) - (0.0F * Block_State.ah[2] + sigma * Block_State.ah[3])) * dt;

  ah[4] = ((-(P3_gain*P3_11) * ddxr_adaptation[2] * s_adaptation[2] + -0.0F * s_adaptation[2] * dx[0] * dx[1]) - (sigma * Block_State.ah[4] + 0.0F * Block_State.ah[5])) * dt;
  ah[5] = ((-0.0F * ddxr_adaptation[2] * s_adaptation[2] + -(P3_gain*P3_22) * s_adaptation[2] * dx[0] * dx[1]) - (0.0F * Block_State.ah[4] + sigma * Block_State.ah[5])) * dt;

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

  if(++log_div >= 5){
    log_div = 0;
    Log_CC0(U[0], U[1], U[2],
      Block_State.x_m[0], Block_State.x_m[1], Block_State.x_m[2],
      Block_State.dx_m[0], Block_State.dx_m[1], Block_State.dx_m[2],
      dxm[1], dxm[3], dxm[5]);

    Log_CC1(Block_State.ah[0], Block_State.ah[1],
      Block_State.ah[2], Block_State.ah[3],
      Block_State.ah[4], Block_State.ah[5],
      error[0], error[1], error[2]);

    Log_CC2(dxr_controller[0], dxr_controller[1], dxr_controller[2],
            ddxr_controller[0], ddxr_controller[1], ddxr_controller[2]);

    Log_CC3(dxr_adaptation[0], dxr_adaptation[1], dxr_adaptation[2],
            ddxr_adaptation[0], ddxr_adaptation[1], ddxr_adaptation[2]);

    Log_CC4(s_controller[0], s_controller[1], s_controller[2], 
            s_adaptation[0], s_adaptation[1], s_adaptation[2]);
  }

}

void Custom_Att_Controller::initialize()
{
  {
    int i;

    // Initialize Conditions for ah integrator
    for (i = 0; i < 6; i++) {
      Block_State.ah[i] = 0.0F;
    }

    // InitializeConditions for x_m integrator
    Block_State.x_m_IC_LOADING = 1.0F;

    // InitializeConditions for dx_m integrator
    Block_State.dx_m_integrator_IC_LOADING = 1.0F;
  }

  // Tuning parameters
  l1 = 23.83F;
  l2 = 25.45F;
  l3 = 17.48F;
  l4 = 19.25F;

  lambda_controller = 1.5F;
  k2 = 0.31F;
  k3 = 0.31F;
  k4 = 0.21F;

  lambda_adaptation = 0.9F;
  P1_gain = 0.10F;
  P1_11 = 0.35F;
  P1_22 = 0.15F;

  P2_gain = 0.1F;
  P2_11 = 0.35F;
  P2_22 = 0.15F;

  P3_gain = 0.08F;
  P3_11 = 0.25F;
  P3_22 = 0.15F;

  sigma = 0.25F;

  prev_yaw_ref = 0.0F;
  prev_yaw_real = 0.0F;
  prev_yaw_model = 0.0F;
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() :
  Block_State()
{

}

Custom_Att_Controller::~Custom_Att_Controller() = default;
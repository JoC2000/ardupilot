#include "Custom_Att_Controller.h"

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

void Custom_Att_Controller::Log_CC3(float s_roll, float s_pitch, float s_yaw) const
{
  struct log_CC3 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC3_MSG),
    time_us     : AP_HAL::micros64(),
    s_roll      : degrees(s_roll),
    s_pitch     : degrees(s_pitch),
    s_yaw       : degrees(s_yaw)
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

uint8_t log_div = 0;

void Custom_Att_Controller::step(
  float dx_d[3], float dx[3], float U[3], float errors[3], float dt,
  float lambdas[4], float k_gains[3], float p_gains[6], float sigma)
{
  int i;
  int i_0;
  float xm[6];
  float dxm[6];
  float ah[6];
  float derror[3];
  float dxr[3];
  float ddxr[3];
  float s[3];
  float temp;

  const float b[18]
  { 0.0F, lambdas[0]*lambdas[0], 0.0F, 0.0F, 0.0F, 0.0F, 
    0.0F, 0.0F, 0.0F, lambdas[1]*lambdas[1], 0.0F, 0.0F, 
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, lambdas[2]*lambdas[2] };

  const float a[36]
  { 0.0F, -(lambdas[0]*lambdas[0]), 0.0F, 0.0F, 0.0F, 0.0F, 
    1.0F, -(lambdas[0]+lambdas[0]), 0.0F, 0.0F, 0.0F, 0.0F, 
    0.0F, 0.0F, 0.0F, -(lambdas[1]*lambdas[1]), 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, -(lambdas[1]+lambdas[1]), 0.0F, 0.0F, 
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -(lambdas[2]*lambdas[2]),
    0.0F, 0.0F, 0.0F, 0.0F, 1.0F, -(lambdas[2]+lambdas[2]) };

  // x_m discrete integrator initial values

  if (Block_State.x_m_IC_LOADING != 0) {
    Block_State.x_m[0] = dx[0];
    Block_State.x_m[1] = dx[1];
    Block_State.x_m[2] = dx[2];
  }

  // dx_m discrete integrator initial values

  if (Block_State.dx_m_integrator_IC_LOADING != 0) {
    Block_State.dx_m[0] = 0;
    Block_State.dx_m[1] = 0;
    Block_State.dx_m[2] = 0;
  }

  // Reference Model
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

    dxm[i] = ((b[i + 6] * dx_d[1] + b[i] * dx_d[0]) + b[i + 12] * dx_d[2]) + temp;
  }
  /*
  Model Reference outputs in State Space form:
  
  dx_m = A*xm + B*x_d

  where:

  dx_d = Desired body rates
  
  Based in body rates
  being p,q,r body rates

  dx_m = [pdot,
          pddot, 
          qdot, 
          qddot,
          rdot,
          rddot]
  */

  // Error definitions
  // error[0] = x[0] - Block_State.x_m[0];
  derror[0] = dx_d[0] - dx[0];

  // error[1] = x[1] - Block_State.x_m[1];
  derror[1] = dx_d[1] - dx[1];

  // error[2] = x[2] - Block_State.x_m[2];
  derror[2] = dx_d[2] - dx[2];
  
  // Sliding surface and xr variables
  dxr[0] = dx_d[0] - lambdas[3] * errors[0];
  ddxr[0] = Block_State.dx_m[0] - lambdas[3] * derror[0];
  s[0] = dx[0] - dxr[0];

  dxr[1] = dx_d[1] - lambdas[3] * errors[1];
  ddxr[1] = Block_State.dx_m[1] - lambdas[3] * derror[1];
  s[1] = dx[1] - dxr[1];

  dxr[2] = dx_d[2] - lambdas[3] * errors[2];
  ddxr[2] = Block_State.dx_m[2] - lambdas[3] * derror[2];
  s[2] = dx[2] - dxr[2];

  // Controller Outputs
  U[0] = -k_gains[0] * s[0] + (ddxr[0] * Block_State.ah[0] + dx[1] * dx[2] * Block_State.ah[1]);  
  U[0] = constrain_float(0.1F*U[0], -10, 10);

  U[1] = -k_gains[1] * s[1] + (ddxr[1] * Block_State.ah[2] + dx[0] * dx[2] * Block_State.ah[3]);
  U[1] = constrain_float(0.1F*U[1], -10, 10);

  U[2] = -k_gains[2] * s[2] + (ddxr[2] * Block_State.ah[4] + dx[0] * dx[1] * Block_State.ah[5]);
  U[2] = constrain_float(0.1F*U[2], -10, 10);

  // Adaptation Law
  ah[0] = ((-p_gains[0] * ddxr[0] * s[0] + -0.0F * s[0] * dx[1] * dx[2]) - (p_gains[0] * sigma * Block_State.ah[0] + 0.0F * Block_State.ah[1])) * dt;
  ah[1] = ((-0.0F * ddxr[0] * s[0] + -p_gains[1] * s[0] * dx[1] * dx[2]) - (0.0F * Block_State.ah[0] + p_gains[1] * sigma * Block_State.ah[1])) * dt;

  ah[2] = ((-p_gains[2] * ddxr[1] * s[1] + -0.0F * s[1] * dx[0] * dx[2]) - (p_gains[2] * sigma * Block_State.ah[2] + 0.0F * Block_State.ah[3])) * dt;
  ah[3] = ((-0.0F * ddxr[1] * s[1] + -p_gains[3] * s[1] * dx[0] * dx[2]) - (0.0F * Block_State.ah[2] + p_gains[3] * sigma * Block_State.ah[3])) * dt;

  ah[4] = ((-p_gains[4] * ddxr[2] * s[2] + -0.0F * s[2] * dx[0] * dx[1]) - (p_gains[4] * sigma * Block_State.ah[4] + 0.0F * Block_State.ah[5])) * dt;
  ah[5] = ((-0.0F * ddxr[2] * s[2] + -p_gains[5] * s[2] * dx[0] * dx[1]) - (0.0F * Block_State.ah[4] + p_gains[5] * sigma * Block_State.ah[5])) * dt;

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

  // if(++log_div >= 5){
    // log_div = 0;
  Log_CC0(U[0], U[1], U[2],
    Block_State.x_m[0], Block_State.x_m[1], Block_State.x_m[2],
    Block_State.dx_m[0], Block_State.dx_m[1], Block_State.dx_m[2],
    dxm[1], dxm[3], dxm[5]);

  Log_CC1(Block_State.ah[0], Block_State.ah[1],
    Block_State.ah[2], Block_State.ah[3],
    Block_State.ah[4], Block_State.ah[5],
    errors[0], errors[1], errors[2]);

  Log_CC2(dxr[0], dxr[1], dxr[2],
          ddxr[0], ddxr[1], ddxr[2]);

  Log_CC3(s[0], s[1], s[2]);
  // }

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
  // lambda_rm = 25.45F;
  // lambda_pm = 25.45F;
  // lambda_ym = 19.25F;

  // lambda_s = 1.5F;

  // Controller gains
  // k1 = 0.31F;
  // k2 = 0.31F;
  // k3 = 0.21F;

  // Adaptation Law gains
  // P1_11 = 0.35F;
  // P1_22 = 0.15F;

  // P2_11 = 0.35F;
  // P2_22 = 0.15F;

  // P3_11 = 0.25F;
  // P3_22 = 0.15F;

  // Sigma modification gain
  // sigma = 0.25F;

  // Initial values
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
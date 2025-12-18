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

void Custom_Att_Controller::step(
  Vector3f wd, Vector3f w, Vector3f &U, Vector3f att_error, float dt,
  Vector3f lambdas_model, Vector3f lambdas_sliding, Vector3f kgains, Vector3f pgains, Vector3f sigma)
{
  Kd.zero(); Lm.zero(); P.zero(), Y.zero(), SigmaM.zero();
  // Define Kd gains
  Kd.a.x = kgains.x;
  Kd.b.y = kgains.y;
  Kd.c.z = kgains.z;

  // Define Lambdas for Reference Model
  Lm.a.x = lambdas_model.x;
  Lm.b.y = lambdas_model.y;
  Lm.c.z = lambdas_model.z;

  // Define Lambdas for Sliding Surface
  Ls.a.x = lambdas_sliding.x;
  Ls.b.y = lambdas_sliding.y;
  Ls.c.z = lambdas_sliding.z;

  // Define Sigma gains
  SigmaM.a.x = sigma.x;
  SigmaM.b.y = sigma.y;
  SigmaM.c.z = sigma.z;

  // Generated Acceleration Reference
  dotw_m = Lm * (wd - wm);

  // Populate P gains matrix
  P.a.x = pgains.x;
  P.b.y = pgains.y;
  P.c.z = pgains.z;

  // Error definitions
  derror = wd - w;
  
  // Sliding surface and wr variables
  wr = wd - (Ls * att_error);
  dwr = dotw_m - (Ls * derror);
  
  s = derror + Ls * att_error;

  // Populate Y matrix
  Y.a.x = dwr.x;
  Y.a.y = -(w.y * w.z);
  Y.a.z = w.y * w.z;

  Y.b.x = w.x * w.z;
  Y.b.y = dwr.y;
  Y.b.z = -(w.x * w.z);

  Y.c.x = -(w.x * w.y);
  Y.c.y = w.x * w.y;
  Y.c.z = dwr.z;

  // Controller Outputs
  U = (Y * a_hat) - (Kd * s);
  
  // Intermediate vector
  ys = Y.transposed() * s;
  da_hat = -(P * ys) - (SigmaM * a_hat);

  // Update adaptation
  a_hat += da_hat * dt;

  // Update acceleration ref
  wm = dotw_m * dt + wm;

  Log_CC0(U.x, U.y, U.z,
    0, 0, 0,
    0, 0, 0,
    dotw_m.x, dotw_m.y, dotw_m.z);

  Log_CC1(a_hat.x, a_hat.y, a_hat.z,
    0, 0, 0,
    att_error.x, att_error.y, att_error.z);

  Log_CC2(wr.x, wr.y, wr.z,
          dwr.x, dwr.y, dwr.z);

  Log_CC3(s.x, s.y, s.z);
}

void Custom_Att_Controller::initialize()
{
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

  wm.zero();
  dotw_m.zero();
  P.zero();
  Kd.zero();
  Lm.zero();
  Ls.zero();
  SigmaM.zero();
  derror.zero();
  wr.zero();
  dwr.zero();
  s.zero();
  Y.zero();
  a_hat.zero();
  da_hat.zero();
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() :
  Block_State()
{

}

Custom_Att_Controller::~Custom_Att_Controller() = default;
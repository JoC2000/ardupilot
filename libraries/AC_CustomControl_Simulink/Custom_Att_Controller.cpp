#include "Custom_Att_Controller.h"

void Custom_Att_Controller::Log_CC0(Vector3f U, Vector3f dwm, Vector3f error, Vector3f d_error) const
{
  struct log_CC0 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC0_MSG),
    time_us     : AP_HAL::micros64(),
    u_roll      : U.x,
    u_pitch     : U.y,
    u_yaw       : U.z,
    dwm1        : dwm.x,
    dwm2        : dwm.y,
    dwm3        : dwm.z,
    err1        : error.x,
    err2        : error.y,
    err3        : error.z,
    derr1       : rate_error.x,
    derr2       : rate_error.y,
    derr3       : rate_error.z,
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC1(Vector3f w_r, Vector3f d_wr, Vector3f w, Vector3f ah) const
{
  struct log_CC1 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC1_MSG),
    time_us     : AP_HAL::micros64(),
    wr1         : wr.x,
    wr2         : wr.y,
    wr3         : wr.z,
    dwr1        : dwr.x,
    dwr2        : dwr.y,
    dwr3        : dwr.z,
    w1          : w.x,
    w2          : w.y,
    w3          : w.z,
    ah1         : ah.x,
    ah2         : ah.y,
    ah3         : ah.z,
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

void Custom_Att_Controller::Log_CC3(Vector3f s_, Vector3f w_d) const
{
  struct log_CC3 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC3_MSG),
    time_us     : AP_HAL::micros64(),
    s_roll      : s.x,
    s_pitch     : s.y,
    s_yaw       : s.z,
    wd1         : w_d.x,
    wd2         : w_d.y,
    wd3         : w_d.z,
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

  // Sliding surface and wr variables
  wr = wd + (Ls * att_error);
  rate_error = wr - w;
  dwr = dotw_m + (Ls * rate_error);
  
  s = rate_error;

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
  U = -(Y * a_hat) + (Kd * s);
  
  // Intermediate vector
  ys = Y.transposed() * s;
  da_hat = -(P * ys) - (SigmaM * a_hat);

  // Update adaptation
  a_hat += da_hat * dt;

  // Update acceleration ref
  wm = dotw_m * dt + wm;

  Log_CC0(U, dotw_m, att_error, rate_error);

  Log_CC1(wr, dwr, w, a_hat);

  Log_CC2(wr.x, wr.y, wr.z,
          dwr.x, dwr.y, dwr.z);

  Log_CC3(s, wd);
}

void Custom_Att_Controller::initialize()
{
  wm.zero();
  dotw_m.zero();
  P.zero();
  Kd.zero();
  Lm.zero();
  Ls.zero();
  SigmaM.zero();
  rate_error.zero();
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
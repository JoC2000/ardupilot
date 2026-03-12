#include "Custom_Att_Controller.h"

void Custom_Att_Controller::Log_CC0(Vector3f U, Vector3f dwm, Vector3f error, Vector3f dah) const
{
  struct log_CC0 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC0_MSG),
    time_us     : AP_HAL::micros64(),
    u_roll      : U.x,
    u_pitch     : U.y,
    u_yaw       : U.z,
    dwm1        : degrees(dwm.x),
    dwm2        : degrees(dwm.y),
    dwm3        : degrees(dwm.z),
    err1        : degrees(error.x),
    err2        : degrees(error.y),
    err3        : degrees(error.z),
    dah1        : dah.x,
    dah2        : dah.y,
    dah3        : dah.z,
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC1(Vector3f w_r, Vector3f d_wr, Vector3f w, Vector3f ah) const
{
  struct log_CC1 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC1_MSG),
    time_us     : AP_HAL::micros64(),
    wr1         : degrees(wr.x),
    wr2         : degrees(wr.y),
    wr3         : degrees(wr.z),
    dwr1        : degrees(dwr.x),
    dwr2        : degrees(dwr.y),
    dwr3        : degrees(dwr.z),
    w1          : degrees(w.x),
    w2          : degrees(w.y),
    w3          : degrees(w.z),
    ah1         : ah.x,
    ah2         : ah.y,
    ah3         : ah.z,
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC2(Vector3f controller_, Vector3f adaptation_) const
{
  struct log_CC2 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC2_MSG),
    time_us     : AP_HAL::micros64(),
    control_r   : controller_.x, 
    control_p   : controller_.y,
    control_y   : controller_.z,
    adapt_r     : adaptation_.x,
    adapt_p     : adaptation_.y,
    adapt_y     : adaptation_.z
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC3(Vector3f s_, Vector3f w_d, Vector3f ys_) const
{
  struct log_CC3 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CC3_MSG),
    time_us     : AP_HAL::micros64(),
    s_roll      : degrees(s.x),
    s_pitch     : degrees(s.y),
    s_yaw       : degrees(s.z),
    wd1         : degrees(w_d.x),
    wd2         : degrees(w_d.y),
    wd3         : degrees(w_d.z),
    ys1         : ys_.x,
    ys2         : ys_.y,
    ys3         : ys_.z
  };
  AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

float Custom_Att_Controller::param_projection(float ahat, float dahat, float ahat_min, float ahat_max)
{
  if (ahat >= ahat_max && dahat > 0.0F) return 0.0F;
  else if (ahat <= ahat_min && dahat < 0.0F) return 0.0F;
  else return dahat;
}

void Custom_Att_Controller::step(
  Vector3f wd, Vector3f w, Vector3f &U, Vector3f att_error, float dt,
  Vector3f lambdas_model, Vector3f lambdas_sliding, Vector3f kgains, Vector3f pgains, Vector3f guesses)
{
  Kd.zero(); Lm.zero(); P.zero(), Y.zero();
  
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
  guess.x = guesses.x;
  guess.y = guesses.y;
  guess.z = guesses.z;

  // Generated Acceleration Reference
  dotw_m = Lm * (wd - wm);

  // Populate P gains matrix
  P.a.x = pgains.x;
  P.b.y = pgains.y;
  P.c.z = pgains.z;

  // Virtual controller reference, includes velocity and attitude targets
  wr = wd + (Ls * att_error);

  // Desired - Actual to match ArduPilot's logic
  s = wr - w;

  // Derivate of the virtual controller reference, the virtual accerelation reference uses the reference model acceleration
  dwr = dotw_m + (Ls * (wd - w));

  // Update acceleration ref
  wm = dotw_m * dt + wm;

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
  adaptation = Y * a_hat;
  controller = Kd * s;

  U = adaptation + controller;
  
  // Intermediate vector
  ys = Y.transposed() * s;
  da_hat = (P * ys);

  // Apply param projection to stop adaptation if not necessary
  da_hat.x = param_projection(a_hat.x, da_hat.x, 0.01F, 0.1F);
  da_hat.y = param_projection(a_hat.y, da_hat.y, 0.01F, 0.1F);
  da_hat.z = param_projection(a_hat.z, da_hat.z, 0.01F, 0.1F);

  // Update adaptation
  a_hat += da_hat * dt;
  a_hat.x = constrain_float(a_hat.x, 0.01F, 0.1F);
  a_hat.y = constrain_float(a_hat.y, 0.01F, 0.1F);
  a_hat.z = constrain_float(a_hat.z, 0.01F, 0.1F);

  Log_CC0(U, dotw_m, att_error, da_hat);
  Log_CC1(wr, dwr, w, a_hat);
  Log_CC2(controller, adaptation);
  Log_CC3(s, wd, ys);
}

void Custom_Att_Controller::initialize()
{
  wm.zero();
  dotw_m.zero();
  P.zero();
  Kd.zero();
  Lm.zero();
  Ls.zero();
  wr.zero();
  dwr.zero();
  s.zero();
  Y.zero();
  a_hat.x = guess.x;
  a_hat.y = guess.y;
  a_hat.z = guess.z;
  da_hat.zero();
  controller.zero();
  adaptation.zero();
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() :
  Block_State()
{

}

Custom_Att_Controller::~Custom_Att_Controller() = default;
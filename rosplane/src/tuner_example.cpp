#include "tuner_example.h"

namespace rosplane
{

tuner_example::tuner_example() : tuner_base()
{
  current_zone = alt_zones::TAKE_OFF;

  c_error_ = 0;
  c_integrator_ = 0;
  r_error_ = 0;
  r_integrator = 0;
  p_error_ = 0;
  p_integrator_ = 0;

}

void tuner_example::control(const params_s &params, const input_s &input, output_s &output)
{
  output.delta_r = 0; //cooridinated_turn_hold(input.beta, params, input.Ts)
  if(input.hold_roll) {
    output.delta_a =  roll_hold(output.phi_c, input.phi, input.p, params, input.Ts);
    // ROS_INFO("Inside roll hold %f", output.delta_a);
  }
  if(input.hold_course) {
    output.phi_c = course_hold(input.chi_c, input.chi, input.phi_ff, input.r, params, input.Ts);
    output.delta_a =  roll_hold(output.phi_c, input.phi, input.p, params, input.Ts);
  }
  if(input.hold_va) {
    output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
  }
  if(input.hold_altitude) {
    ROS_INFO("ALTITUDE HOLD");
    output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
    output.theta_c = altitiude_hold(input.h_c, input.h, params, input.Ts);
    output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
  }
  else if(input.hold_vh) {
    ROS_INFO("VERTICAL RATE HOLD");
    output.theta_c = vertical_rate_hold(input.vh_c, input.vh, params, input.Ts);
    output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
    output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
  }
  else if(input.hold_pitch) {
    output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
    // ROS_INFO("Inside pitch hold %f", output.delta_e);
  } 
  // output.phi_c = course_hold(input.chi_c, input.chi, input.phi_ff, input.r, params, input.Ts);
  // output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params, input.Ts);

  // output.current_zone = current_zone;
  // output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);

  // Reset the errors 
  // False means we should also reset the errors
  if(!input.hold_roll && !input.hold_course) {
    r_error_ = 0;
    r_integrator = 0;
  }
  if(!input.hold_pitch && !input.hold_altitude && !input.hold_vh) {
    p_error_ = 0;
    p_integrator_ = 0;
  }
  if(!input.hold_course) {
    c_error_ = 0;
    c_integrator_ = 0;
  }
  if(!input.hold_altitude) {
    a_error_ = 0;
    a_integrator_ = 0;
    a_differentiator_ = 0;

    at_error_ = 0;
    at_integrator_ = 0;
    at_differentiator_ = 0;
  }
  if(!input.hold_vh) {
    vh_error_ = 0;
    vh_integrator_ = 0;
    vh_differentiator_ = 0;
  }
}

float tuner_example::course_hold(float chi_c, float chi, float phi_ff, float r, const params_s &params, float Ts)
{
  float error = chi_c - chi;

  c_integrator_ = c_integrator_ + (Ts/2.0)*(error + c_error_);

  float up = params.c_kp*error;
  float ui = params.c_ki*c_integrator_;
  float ud = params.c_kd*r;

  float phi_c = sat(up + ui + ud + phi_ff, 30.0*3.14/180.0, -30.0*3.14/180.0);
  if (fabs(params.c_ki) >= 0.00001)
  {
    float phi_c_unsat = up + ui + ud + phi_ff;
    c_integrator_ = c_integrator_ + (Ts/params.c_ki)*(phi_c - phi_c_unsat);
  }

  c_error_ = error;
  return phi_c;
}

float tuner_example::roll_hold(float phi_c, float phi, float p, const params_s &params, float Ts)
{
  float error = phi_c - phi;

  r_integrator = r_integrator + (Ts/2.0)*(error + r_error_);

  float up = params.r_kp*error;
  float ui = params.r_ki*r_integrator;
  float ud = params.r_kd*p;

  float delta_a = sat(up + ui + ud, params.max_a, -params.max_a);
  if (fabs(params.r_ki) >= 0.00001)
  {
    float delta_a_unsat = up + ui + ud;
    r_integrator = r_integrator + (Ts/params.r_ki)*(delta_a - delta_a_unsat);
  }

  r_error_ = error;
  return delta_a;
}

float tuner_example::pitch_hold(float theta_c, float theta, float q, const params_s &params, float Ts)
{
  float error = theta_c - theta;

  p_integrator_ = p_integrator_ + (Ts/2.0)*(error + p_error_);

  float up = params.p_kp*error;
  float ui = params.p_ki*p_integrator_;
  float ud = params.p_kd*q;

  float delta_e = sat(params.trim_e/params.pwm_rad_e + up + ui + ud, params.max_e, -params.max_e);
  if (fabs(params.p_ki) >= 0.00001)
  {
    float delta_e_unsat = params.trim_e/params.pwm_rad_e + up + ui + ud;
    p_integrator_ = p_integrator_ + (Ts/params.p_ki)*(delta_e - delta_e_unsat);
  }

  p_error_ = error;
  ROS_INFO("%f, %f", error, p_integrator_);
  return delta_e;
}

float tuner_example::airspeed_with_pitch_hold(float Va_c, float Va, const params_s &params, float Ts)
{
  float error = Va_c - Va;

  ap_integrator_ = ap_integrator_ + (Ts/2.0)*(error + ap_error_);
  ap_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*ap_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - ap_error_);

  float up = params.a_p_kp*error;
  float ui = params.a_p_ki*ap_integrator_;
  float ud = params.a_p_kd*ap_differentiator_;

  float theta_c = sat(up + ui + ud, 10.0*3.14/180.0, -10.0*3.14/180.0);
  if (fabs(params.a_p_ki) >= 0.00001)
  {
    float theta_c_unsat = up + ui + ud;
    ap_integrator_ = ap_integrator_ + (Ts/params.a_p_ki)*(theta_c - theta_c_unsat);
  }

  ap_error_ = error;
  return theta_c;
}

float tuner_example::airspeed_with_throttle_hold(float Va_c, float Va, const params_s &params, float Ts)
{
  float error = Va_c - Va;

  at_integrator_ = at_integrator_ + (Ts/2.0)*(error + at_error_);
  at_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*at_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - at_error_);

  float up = params.vh_kp*error;
  float ui = params.vh_ki*at_integrator_;
  float ud = params.vh_kd*at_differentiator_;

  float delta_t = sat(params.trim_t + up + ui + ud, params.max_t, 0);
  if (fabs(params.vh_ki) >= 0.00001)
  {
    float delta_t_unsat = params.trim_t + up + ui + ud;
    at_integrator_ = at_integrator_ + (Ts/params.vh_ki)*(delta_t - delta_t_unsat);
  }

  at_error_ = error;
  return delta_t;
}

float tuner_example::altitiude_hold(float h_c, float h, const params_s &params, float Ts)
{
  float error = h_c - h;

  // ROS_INFO("Altitude Hold : %f, %f, %f", error, a_integrator_, a_error_);
  a_integrator_ = a_integrator_ + (Ts/2.0)*(error + a_error_);
  a_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*a_differentiator_ + (2.0 /
                      (2.0*params.tau + Ts))*(error - a_error_);

  float up = params.a_kp*error;
  float ui = params.a_ki*a_integrator_;
  float ud = params.a_kd*a_differentiator_;

    ROS_INFO("Altitude Hold : %f, %f", error, ud);

  float theta_c = sat(up + ui + ud, 10.0*3.14/180.0, -10.0*3.14/180.0);
  if (fabs(params.a_ki) >= 0.00001)
  {
    // ROS_INFO("Before windup: %f, %f, %f", error, a_integrator_, a_error_);
    float theta_c_unsat = up + ui + ud;
    a_integrator_ = a_integrator_ + (Ts/params.a_ki)*(theta_c - theta_c_unsat);
    // ROS_INFO("After windup : %f, %f, %f", error, a_integrator_, a_error_);
  }

  a_error_ = error; // rosplane people has written at_error_ here ... huge confusion
  // ROS_INFO("Altitude Hold : %f, %f, %f", error, a_integrator_, a_error_);
  // ROS_INFO("Altitude hold error : %f \n up : %f, ud : %f, ui : %f \n theta_c : %f", error, up, ud, ui, theta_c);
  return theta_c;
}

//float tuner_example::cooridinated_turn_hold(float v, const params_s &params, float Ts)
//{
//    //todo finish this if you want...
//    return 0;
//}

float tuner_example::vertical_rate_hold(float vh_c, float vh, const params_s &params, float Ts)
{
  float error = vh_c - vh;

  vh_integrator_ = vh_integrator_ + (Ts/2.0)*(error + vh_error_);
  vh_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*vh_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - vh_error_);

  float up = params.vh_kp*error;
  float ui = params.vh_ki*vh_integrator_;
  float ud = params.vh_kd*vh_differentiator_;

  float theta_c = sat( up + ui + ud, 10.0*3.14/180, -10.0*3.14/180);
  if (fabs(params.vh_ki) >= 0.00001)
  {
    float theta_c_unsat =  up + ui + ud;
    vh_integrator_ = vh_integrator_ + (Ts/params.vh_ki)*(theta_c - theta_c_unsat);
  }

  vh_error_ = error;
  return theta_c;
}

float tuner_example::sat(float value, float up_limit, float low_limit)
{
  float rVal;
  if (value > up_limit)
    rVal = up_limit;
  else if (value < low_limit)
    rVal = low_limit;
  else
    rVal = value;

  return rVal;
}

} //end namespace

#include "controller_example.h"


namespace rosplane
{

controller_example::controller_example() : controller_base()
{
  current_zone = alt_zones::TAKE_OFF;

  c_error_ = 0;
  c_integrator_ = 0;
  r_error_ = 0;
  r_integrator = 0;
  p_error_ = 0;
  p_integrator_ = 0;

  ct_error_ = 0;
  ct_integrator_ = 0;
  ct_differentiator_ = 0;

  t_error_ = 0;
  t_integrator_ = 0;

  vh_differentiator_ = 0;
  vh_error_ = 0;
  vh_integrator_ = 0;

  throttle_ramp_ = 0.0; // only for the purpose of takeoff
}

void controller_example::control(const params_s &params, const input_s &input, output_s &output)
{
  output.delta_r = cooridinated_turn_hold(input.beta, params, input.Ts);
  output.phi_c = course_hold(input.chi_c, input.chi, input.phi_ff, input.r, params, input.Ts);
  output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params, input.Ts);
  
  float e_py = -sinf(input.psi - chi_0) * sqrt(input.pn*input.pn + input.pe*input.pe);
  float psi_c = input.psi - params.chi_infi * 2/M_PI_F * atanf(params.k_path * e_py);
  output.psi_c = chi_0;

  if(input.land) {
    current_zone = alt_zones::LAND;
  }

  // Use this if code block if spawning in air 
  if(params.ai_mode) {
    accept_ai_commands_ = true;
    current_zone = alt_zones::AI_MODE;
        
    // ROS_INFO("GOING TO AI MODE");
  } 

  switch (current_zone)
  {
  case alt_zones::TAKE_OFF:
    output.phi_c = 0;
    output.delta_a = roll_hold(0.0, input.phi, input.p, params, input.Ts);

    if(throttle_ramp_<=0.5) {
      throttle_ramp_ += 0.001;
    }
    else {
      throttle_ramp_ += 0.001;
    }

    throttle_ramp_ = sat(throttle_ramp_, params.max_t, 0.0);
    output.delta_t =  throttle_ramp_;
    // output.delta_t = params.max_t;
    output.theta_c = 5.0*3.14/180.0;
    if(input.va < 25) output.theta_c = 0.0 * 3.14/180.0; // takeoff only when speed is good

    output.delta_r = takeoff_path_hold(output.psi_c, input.psi, input.r, params, input.Ts);
    // ROS_INFO("TAKING OFF %f, %f", output.delta_r, output.theta_c);

    if (input.h >= params.alt_toz)
    {
      ROS_DEBUG("climb");
      current_zone = alt_zones::CLIMB;
      ap_error_ = 0;
      ap_integrator_ = 0;
      ap_differentiator_ = 0;
      if(params.ai_mode) {
        accept_ai_commands_ = true;
        current_zone = alt_zones::AI_MODE;
        ROS_INFO("GOING TO AI MODE");
      }
    }
    break;
  case alt_zones::CLIMB:
    ROS_INFO("CLIMB");
    output.delta_t = params.max_t;
    output.theta_c = airspeed_with_pitch_hold(input.Va_c, input.va, params, input.Ts);
    if (input.h >= input.h_c - params.alt_hz)
    {
      // ROS_DEBUG("hold");
      current_zone = alt_zones::ALTITUDE_HOLD;
      at_error_ = 0;
      at_integrator_ = 0;
      at_differentiator_ = 0;
      a_error_ = 0;
      a_integrator_ = 0;
      a_differentiator_ = 0;
    }
    else if (input.h <= params.alt_toz)
    {
      // ROS_DEBUG("takeoff");
      current_zone = alt_zones::TAKE_OFF;
    }
    break;
  case alt_zones::DESCEND:
    // ROS_INFO("DESCEND");
    output.delta_t = 0;
    output.theta_c = airspeed_with_pitch_hold(input.Va_c, input.va, params, input.Ts);
    if (input.h <= input.h_c + params.alt_hz)
    {
      // ROS_DEBUG("hold");
      current_zone = alt_zones::ALTITUDE_HOLD;
      at_error_ = 0;
      at_integrator_ = 0;
      at_differentiator_ = 0;
      a_error_ = 0;
      a_integrator_ = 0;
      a_differentiator_ = 0;
    }
    break;
  case alt_zones::ALTITUDE_HOLD:
    ROS_INFO("ALTITUDE_HOLD");
    output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
    output.theta_c = altitiude_hold(input.h_c, input.h, params, input.Ts);
    if (input.h >= input.h_c + params.alt_hz)
    {
      // ROS_DEBUG("desend");
      current_zone = alt_zones::DESCEND;
      ap_error_ = 0;
      ap_integrator_ = 0;
      ap_differentiator_ = 0;
    }
    else if (input.h <= input.h_c - params.alt_hz)
    {
      // ROS_DEBUG("climb");
      current_zone = alt_zones::CLIMB;
      ap_error_ = 0;
      ap_integrator_ = 0;
      ap_differentiator_ = 0;
    }
    break;
    
  case alt_zones::LAND:
    ROS_INFO("LANDING");
    output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
    output.theta_c = vertical_rate_hold(input.vh_c, input.vh, params, input.Ts);
    break;
  
  case alt_zones::AI_MODE:
    // If you want to test your own Learning agent, it's probably best to make changes in here 
    output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
    output.theta_c = vertical_rate_hold(input.vh_c, input.vh, params, input.Ts);
    output.delta_a = roll_hold(input.phi_c, input.phi, input.p, params, input.Ts);    // we use input.phi_c as commanded roll 
    break;

  default:
    break;
  }

  output.current_zone = current_zone;
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

float controller_example::course_hold(float chi_c, float chi, float phi_ff, float r, const params_s &params, float Ts)
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

float controller_example::roll_hold(float phi_c, float phi, float p, const params_s &params, float Ts)
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

float controller_example::pitch_hold(float theta_c, float theta, float q, const params_s &params, float Ts)
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
  return delta_e;
}

float controller_example::airspeed_with_pitch_hold(float Va_c, float Va, const params_s &params, float Ts)
{
  float error = Va_c - Va;

  ap_integrator_ = ap_integrator_ + (Ts/2.0)*(error + ap_error_);
  ap_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*ap_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - ap_error_);

  float up = params.a_p_kp*error;
  float ui = params.a_p_ki*ap_integrator_;
  float ud = params.a_p_kd*ap_differentiator_;

  float theta_c = sat(up + ui + ud, 15.0*3.14/180.0, -15.0*3.14/180.0);
  if (fabs(params.a_p_ki) >= 0.00001)
  {
    float theta_c_unsat = up + ui + ud;
    ap_integrator_ = ap_integrator_ + (Ts/params.a_p_ki)*(theta_c - theta_c_unsat);
  }
  ap_error_ = error;
  return theta_c;
}

float controller_example::airspeed_with_throttle_hold(float Va_c, float Va, const params_s &params, float Ts)
{
  float error = Va_c - Va;

  at_integrator_ = at_integrator_ + (Ts/2.0)*(error + at_error_);
  at_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*at_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - at_error_);

  float up = params.a_t_kp*error;
  float ui = params.a_t_ki*at_integrator_;
  float ud = params.a_t_kd*at_differentiator_;
  float uff = params.a_t_ff*Va_c; // add feedforward term

  float delta_t = sat(params.trim_t + up + ui + ud + uff, params.max_t, 0);
  if (fabs(params.a_t_ki) >= 0.00001)
  {
    float delta_t_unsat = params.trim_t + up + ui + ud + uff;
    at_integrator_ = at_integrator_ + (Ts/params.a_t_ki)*(delta_t - delta_t_unsat);
  }

  ROS_INFO("Airspeed with throttle : %f, %f, %f, %f", params.a_t_kp, params.a_t_ki, params.a_t_kd, params.a_t_ff );
  at_error_ = error;
  return delta_t;
}

float controller_example::altitiude_hold(float h_c, float h, const params_s &params, float Ts)
{
  float error = h_c - h;

  a_integrator_ = a_integrator_ + (Ts/2.0)*(error + a_error_);
  a_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*a_differentiator_ + (2.0 /
                      (2.0*params.tau + Ts))*(error - a_error_);

  float up = params.a_kp*error;
  float ui = params.a_ki*a_integrator_;
  float ud = params.a_kd*a_differentiator_;

  float theta_c = sat(up + ui + ud, 5.0*3.14/180.0, -5.0*3.14/180.0); // change max and min from +- 15 to 5 degrees
  if (fabs(params.a_ki) >= 0.00001)
  {
    float theta_c_unsat = up + ui + ud;
    a_integrator_ = a_integrator_ + (Ts/params.a_ki)*(theta_c - theta_c_unsat);
  }

  a_error_ = error;
  return theta_c;
}

float controller_example::cooridinated_turn_hold(float v, const params_s &params, float Ts)
{
   //todo finish this if you want...
   float beta_c = 0.0;
   float error = beta_c - v;

   ct_integrator_ = ct_integrator_ + (Ts/ 2.0)*(error + ct_error_);
   ct_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*ct_differentiator_ + (2.0 /
                      (2.0*params.tau + Ts))*(error - ct_error_);

   float up = params.b_kp*error;
   float ui = params.b_ki*ct_integrator_;
   float ud = params.b_kd*ct_differentiator_;

   float delta_r = sat(up + ui + ud, params.max_r, -params.max_r);
   if(fabs(params.b_ki) >= 0.00001) {
     float delta_r_unsat = up + ui + ud;
     ct_integrator_ = ct_integrator_ + (Ts/params.b_ki)*(delta_r - delta_r_unsat);
   }

   ct_error_ = error;
  //  ROS_INFO("coordinated turn %f %f %f %f", delta_r, ct_error_, ct_integrator_, ct_differentiator_);
  //  ROS_INFO("params %f %f %f", params.b_kp, params.b_kd, params.b_ki);
   return delta_r;
  
  // float delta_r;
  // if(v > 0) delta_r = -0.1;
  // else delta_r = 0.1;
  // delta_r = sat(delta_r, 0.09, -0.09);
  // ROS_INFO("beta %f %f",v,delta_r );
  // return delta_r;
} 

float controller_example::takeoff_path_hold(float psi_c, float psi, float r, const struct params_s &params, float Ts)
{
  float error = psi_c - psi;

  t_integrator_ = t_integrator_ + (Ts/2.0)*(error + t_error_);

  float up = params.t_kp*error;
  float ui = params.t_ki*t_integrator_;
  float ud = params.t_kd*r;
  // ROS_INFO("%f", params.t_r_ff);
  float delta_r = sat(up + ui + ud + params.t_r_ff, params.max_r, -params.max_r);
  if (fabs(params.t_ki) >= 0.00001)
  {
    float delta_r_unsat = up + ui + ud;
    t_integrator_ = t_integrator_ + (Ts/params.t_ki)*(delta_r - delta_r_unsat);
  }

  t_error_ = error;
  return delta_r;
}

float controller_example::vertical_rate_hold(float vh_c, float vh, const params_s &params, float Ts)
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

float controller_example::sat(float value, float up_limit, float low_limit)
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

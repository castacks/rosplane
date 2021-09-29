#include "path_follower_example.h"


namespace rosplane
{

path_follower_example::path_follower_example()
{
  alpha = 0;
}

void path_follower_example::follow(const params_s &params, const input_s &input, output_s &output)
{
  output.land = false; // Initially landing sequence is off

  ROS_INFO("PATH FOLLOWING");
  if (input.p_type == path_type::Line) // follow straight line path specified by r and q
  {
    // compute wrapped version of the path angle
    float chi_q = atan2f(input.q_path[1], input.q_path[0]);
    while (chi_q - input.chi < -M_PI)
      chi_q += 2.0*M_PI;
    while (chi_q - input.chi > M_PI)
      chi_q -= 2.0*M_PI;

    float path_error = -sinf(chi_q)*(input.pn - input.r_path[0]) + cosf(chi_q)*(input.pe - input.r_path[1]);
    // heading command
    output.chi_c = chi_q - params.chi_infty*2/M_PI*atanf(params.k_path*path_error);

    // desired altitude
    float h_d = -input.r_path[2] - sqrtf(powf((input.r_path[0] - input.pn), 2) + powf((input.r_path[1] - input.pe),
                                         2))*(input.q_path[2])/sqrtf(powf(input.q_path[0], 2) + powf(input.q_path[1], 2));
    // commanded altitude is desired altitude
    output.h_c = h_d;
    output.phi_ff = 0.0;
  }
  else // follow a orbit path specified by c_orbit, rho_orbit, and lam_orbit
  {
    float d = sqrtf(powf((input.pn - input.c_orbit[0]), 2) + powf((input.pe - input.c_orbit[1]),
                    2)); // distance from orbit center
    // compute wrapped version of angular position on orbit
    float varphi = atan2f(input.pe - input.c_orbit[1], input.pn - input.c_orbit[0]);
    while ((varphi - input.chi) < -M_PI)
      varphi += 2.0*M_PI;
    while ((varphi - input.chi) > M_PI)
      varphi -= 2.0*M_PI;
    //compute orbit error
    float norm_orbit_error = (d - input.rho_orbit)/input.rho_orbit;
    output.chi_c = varphi + input.lam_orbit*(M_PI/2.0 + atanf(params.k_orbit*norm_orbit_error));

    // commanded altitude is the height of the orbit
    float h_d = -input.c_orbit[2];
    output.h_c = h_d;
    output.phi_ff = (norm_orbit_error < 0.5 ? input.lam_orbit*atanf(input.Va*input.Va/(9.8*input.rho_orbit)) : 0);
  }
  output.Va_c = input.Va_d;

  if(input.land) 
  {
    float k_path = 0.001;
    ROS_INFO("FOLLOWING RUNWAY DIRECTION FOR LANDING");
    float chi_q = params.chi_0;
    while (chi_q - input.chi < -M_PI)
      chi_q += 2.0*M_PI;
    while (chi_q - input.chi > M_PI)
      chi_q -= 2.0*M_PI;

    // we use the negative sign in (input.pn - 0) because r is the starting of the path but in this case (0,0) is the ending of the path
    float rn = -20000.0 * cosf(chi_q);
    float re = -20000.0 * sinf(chi_q);
    float path_error = -sinf(chi_q)*((input.pn - rn)) + cosf(chi_q)*((input.pe - re));
    float dist_to_runway = sqrt(powf((input.pn - 50*cosf(chi_q)), 2) + powf((input.pe- 50*sinf(chi_q)),2));
    // heading command
    if(dist_to_runway<1500) {
      k_path = 0.005;
    }

    
    float t = (dist_to_runway + 0.00001)/input.Vg;
    // output.vh = std::max(-(input.h)/t, (float)-0.5); // max descend rate should be 1 m/s == 196 fpm
    output.vh = -input.h/t;
    output.Va_c  = 30;

    ROS_INFO("Path error : %f", path_error);
    if(dist_to_runway < input.h/tanf(4.5*M_PI/180.0)) {
      output.land = true;
      k_path = 0.0025;
      output.Va_c = 25;
      alpha = std::min(1.0, alpha+0.001);
      output.vh = alpha*output.vh;
      ROS_INFO("FOLLOWER : GOING INTO LANDING, %f", alpha);
    }

    if(dist_to_runway < 1000) {
      output.Va_c = 25;
    }

    if(input.h < 10) {
      output.Va_c = 0;
    }
    // if(input.h < 50) {
    //   output.vh = std::max((float)-100.0, (float)output.vh);
    // }
    // if(dist_to_runway < 300) {
    //   output.vh = -2;  // m/s
    // }

    output.chi_c = chi_q - params.chi_infty*2/M_PI*atanf(k_path*path_error);
  }
}

} //end namespace

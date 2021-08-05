#include "tuner_base.h"
#include "tuner_example.h"

#define M_PI_F 3.14159265358979323846f
float DEG_2_RAD = M_PI_F/180.0;

namespace rosplane
{

tuner_base::tuner_base():
  nh_(),
  nh_private_("~")
{
  vehicle_state_sub_ = nh_.subscribe("state", 10, &tuner_base::vehicle_state_callback, this);

  controller_commands_sub_ = nh_.subscribe("controller_commands", 10, &tuner_base::controller_commands_callback,
                             this);
  
  tuner_commands_sub_ = nh_.subscribe("tuner_commands", 10, &tuner_base::tuner_commands_callback, this);

  memset(&vehicle_state_, 0, sizeof(vehicle_state_));
  memset(&controller_commands_, 0, sizeof(controller_commands_));

  nh_private_.param<double>("TRIM_E", params_.trim_e, 0.0);
  nh_private_.param<double>("TRIM_A", params_.trim_a, 0.0);
  nh_private_.param<double>("TRIM_R", params_.trim_r, 0.0);
  nh_private_.param<double>("TRIM_T", params_.trim_t, 0.6);
  nh_private_.param<double>("PWM_RAD_E", params_.pwm_rad_e, 1.0);
  nh_private_.param<double>("PWM_RAD_A", params_.pwm_rad_a, 1.0);
  nh_private_.param<double>("PWM_RAD_R", params_.pwm_rad_r, 1.0);
  nh_private_.param<double>("ALT_TOZ", params_.alt_toz, 20.0);
  nh_private_.param<double>("ALT_HZ", params_.alt_hz, 10.0);
  nh_private_.param<double>("TAU", params_.tau, 5.0);
  nh_private_.param<double>("COURSE_KP", params_.c_kp, 0.7329);
  nh_private_.param<double>("COURSE_KD", params_.c_kd, 0.0);
  nh_private_.param<double>("COURSE_KI", params_.c_ki, 0.0);
  nh_private_.param<double>("ROLL_KP", params_.r_kp, 30.0);
  nh_private_.param<double>("ROLL_KD", params_.r_kd, -0.945);
  nh_private_.param<double>("ROLL_KI", params_.r_ki, 0.0);//0.10f);
  nh_private_.param<double>("PITCH_KP", params_.p_kp, 1.0);
  nh_private_.param<double>("PITCH_KD", params_.p_kd, -0.17);
  nh_private_.param<double>("PITCH_KI", params_.p_ki, 0.0);
  nh_private_.param<double>("PITCH_FF", params_.p_ff, 0.0);
  nh_private_.param<double>("AS_PITCH_KP", params_.a_p_kp, -0.0713);
  nh_private_.param<double>("AS_PITCH_KD", params_.a_p_kd, -0.0635);
  nh_private_.param<double>("AS_PITCH_KI", params_.a_p_ki, 0.0);
  nh_private_.param<double>("AS_THR_KP", params_.a_t_kp, 3.2);
  nh_private_.param<double>("AS_THR_KD", params_.a_t_kd, 0.0);
  nh_private_.param<double>("AS_THR_KI", params_.a_t_ki, 0.0);
  nh_private_.param<double>("ALT_KP", params_.a_kp, 0.045);
  nh_private_.param<double>("ALT_KD", params_.a_kd, 0.0);
  nh_private_.param<double>("ALT_KI", params_.a_ki, 0.01);
  nh_private_.param<double>("BETA_KP", params_.b_kp, -0.1164);
  nh_private_.param<double>("BETA_KD", params_.b_kd, 0.0);
  nh_private_.param<double>("BETA_KI", params_.b_ki, -0.0037111);
  nh_private_.param<double>("VH_KP", params_.vh_kp, 0.001);          // params for Vertical rate hold
  nh_private_.param<double>("VH_KD", params_.vh_kd, 0.0);
  nh_private_.param<double>("VH_KI", params_.vh_ki, 0.0);
  nh_private_.param<double>("MAX_E", params_.max_e, 0.610);
  nh_private_.param<double>("MAX_A", params_.max_a, 0.523);
  nh_private_.param<double>("MAX_R", params_.max_r, 0.523);
  nh_private_.param<double>("MAX_T", params_.max_t, 1.0);

  func_ = boost::bind(&tuner_base::reconfigure_callback, this, _1, _2);
  server_.setCallback(func_);

  actuators_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 10);
  internals_pub_ = nh_.advertise<rosplane_msgs::Controller_Internals>("controller_inners", 10);
  act_pub_timer_ = nh_.createTimer(ros::Duration(1.0/100.0), &tuner_base::actuator_controls_publish, this);
  commanded_values_pub_ = nh_.advertise<rosplane_msgs::Commanded_Values>("commanded_values", 10);

  command_recieved_ = false;
  angle_in_deg_ = 1;
}

void tuner_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
{
  vehicle_state_ = *msg;
  if(angle_in_deg_) {
    vehicle_state_.alpha = vehicle_state_.alpha * DEG_2_RAD;
    vehicle_state_.beta = vehicle_state_.beta * DEG_2_RAD;
    vehicle_state_.phi = vehicle_state_.phi * DEG_2_RAD;
    vehicle_state_.theta = vehicle_state_.theta * DEG_2_RAD;
    vehicle_state_.psi = vehicle_state_.psi * DEG_2_RAD; 
    vehicle_state_.chi =vehicle_state_.chi * DEG_2_RAD;
  }
}

void tuner_base::controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr &msg)
{
  command_recieved_ = true;
  controller_commands_ = *msg;
}

void tuner_base::tuner_commands_callback(const rosplane_msgs::Tuner_CommandsConstPtr &msg) 
{
  command_recieved_ = true;
  tuner_commands_ = *msg;
  if(angle_in_deg_) {
    tuner_commands_.phi_c = tuner_commands_.phi_c * DEG_2_RAD;
    tuner_commands_.theta_c = tuner_commands_.theta_c * DEG_2_RAD;
    tuner_commands_.chi_c = tuner_commands_.chi_c * DEG_2_RAD;
  }
}

void tuner_base::reconfigure_callback(rosplane::TunerConfig &config, uint32_t level)
{
  params_.trim_e = config.TRIM_E;
  params_.trim_a = config.TRIM_A;
  params_.trim_r = config.TRIM_R;
  params_.trim_t = config.TRIM_T;

  params_.c_kp = config.COURSE_KP;
  params_.c_kd = config.COURSE_KD;
  params_.c_ki = config.COURSE_KI;

  params_.r_kp = config.ROLL_KP;
  params_.r_kd = config.ROLL_KD;
  params_.r_ki = config.ROLL_KI;

  params_.p_kp = config.PITCH_KP;
  params_.p_kd = config.PITCH_KD;
  params_.p_ki = config.PITCH_KI;
  params_.p_ff = config.PITCH_FF;

  params_.a_p_kp = config.AS_PITCH_KP;
  params_.a_p_kd = config.AS_PITCH_KD;
  params_.a_p_ki = config.AS_PITCH_KI;

  params_.a_t_kp = config.AS_THR_KP;
  params_.a_t_kd = config.AS_THR_KD;
  params_.a_t_ki = config.AS_THR_KI;

  params_.a_kp = config.ALT_KP;
  params_.a_kd = config.ALT_KD;
  params_.a_ki = config.ALT_KI;

  params_.b_kp = config.BETA_KP;
  params_.b_kd = config.BETA_KD;
  params_.b_ki = config.BETA_KI;

  params_.vh_kp = config.VH_KP;
  params_.vh_kd = config.VH_KD;
  params_.vh_ki = config.VH_KI;
}

void tuner_base::convert_to_pwm(tuner_base::output_s &output)
{
  output.delta_e = output.delta_e*params_.pwm_rad_e;
  output.delta_a = output.delta_a*params_.pwm_rad_a;
  output.delta_r = output.delta_r*params_.pwm_rad_r;
}

void tuner_base::actuator_controls_publish(const ros::TimerEvent &)
{
  struct input_s input;
  input.h = -vehicle_state_.position[2];
  input.va = vehicle_state_.Va;
  input.phi = vehicle_state_.phi;
  input.theta = vehicle_state_.theta;
  input.chi = vehicle_state_.chi;
  input.p = vehicle_state_.p;
  input.q = vehicle_state_.q;
  input.r = vehicle_state_.r;
  input.Va_c = tuner_commands_.Va_c;
  input.h_c = tuner_commands_.h_c;
  input.chi_c = tuner_commands_.chi_c;
  input.phi_ff = tuner_commands_.phi_ff;
  input.Ts = 0.01f;
  input.vh = vehicle_state_.vh * 0.305/60.0;
  input.vh_c = tuner_commands_.vh_c * 0.305/60.0;

  // Variables added for tuning
  input.hold_roll = tuner_commands_.hold_roll;
  input.hold_pitch = tuner_commands_.hold_pitch;
  input.hold_course =  tuner_commands_.hold_course;
  input.hold_altitude = tuner_commands_.hold_altitude;
  input.hold_vh = tuner_commands_.hold_vh;
  input.hold_va = tuner_commands_.hold_va;

  struct output_s output;
  output.phi_c = tuner_commands_.phi_c;
  output.theta_c = tuner_commands_.theta_c;
  output.delta_a = output.delta_e = output.delta_r = output.delta_t = -1.1;
  if (command_recieved_ == true)
  {
    control(params_, input, output);

    convert_to_pwm(output);

    rosflight_msgs::Command actuators;
    /* publish actuator controls */
    // Add header stamp to allow plotting
    actuators.header.stamp = ros::Time::now();
    actuators.ignore = 0;
    actuators.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
    actuators.x = output.delta_a;//(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    actuators.y = output.delta_e;//(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    actuators.z = output.delta_r;//(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    actuators.F = output.delta_t;//(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

    actuators_pub_.publish(actuators);

    commanded_values_.phi_c = output.phi_c;
    commanded_values_.theta_c = output.theta_c;
    commanded_values_.Va_c = input.Va_c;
    commanded_values_.h_c = -input.h_c;                 // Easier to understand since position is in NED
    commanded_values_.chi_c = tuner_commands_.chi_c;
    commanded_values_.vh_c = tuner_commands_.vh_c;
    commanded_values_.Va_c = tuner_commands_.Va_c;
    // commanded_values_.psi_c = output.psi_c;
    // commanded_values_.chi_0 = chi_0;

    if(angle_in_deg_) {
      // ROS_INFO("Converting angle to degrees");
      // std::cout<<commanded_values_.theta_c<<" ";

      commanded_values_.phi_c = commanded_values_.phi_c * 1/DEG_2_RAD;
      commanded_values_.theta_c = commanded_values_.theta_c * 1/DEG_2_RAD;
      commanded_values_.chi_c = commanded_values_.chi_c * 1/DEG_2_RAD;
      // commanded_values_.psi_c = commanded_values_.psi_c * 1/DEG_2_RAD;
      // commanded_values_.chi_0 = commanded_values_.chi_0 * 1/DEG_2_RAD;
      
      // std::cout<<commanded_values_.theta_c<<"\n";
    }

    commanded_values_pub_.publish(commanded_values_);

    // if (internals_pub_.getNumSubscribers() > 0)
    // {
    //   rosplane_msgs::Controller_Internals inners;
    //   inners.phi_c = output.phi_c;
    //   inners.theta_c = output.theta_c;
    //   switch (output.current_zone)
    //   {
    //   case alt_zones::TAKE_OFF:
    //     inners.alt_zone = inners.ZONE_TAKE_OFF;
    //     break;
    //   case alt_zones::CLIMB:
    //     inners.alt_zone = inners.ZONE_CLIMB;
    //     break;
    //   case alt_zones::DESCEND:
    //     inners.alt_zone = inners.ZONE_DESEND;
    //     break;
    //   case alt_zones::ALTITUDE_HOLD:
    //     inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
    //     break;
    //   default:
    //     break;
    //   }
    //   inners.aux_valid = false;
    //   internals_pub_.publish(inners);
    // }
  }
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_tuner_controller");
  rosplane::tuner_base *cont = new rosplane::tuner_example();

  ros::spin();

  return 0;
}

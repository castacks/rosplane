#include "controller_base.h"
#include "controller_example.h"

float DEG_2_RAD = M_PI_F/180.0;
float MS_TO_FPM = 60.0/0.305;

namespace rosplane
{

controller_base::controller_base():
  nh_(),
  nh_private_("~")
{
  vehicle_state_sub_ = nh_.subscribe("state", 10, &controller_base::vehicle_state_callback, this);
  controller_commands_sub_ = nh_.subscribe("controller_commands", 10, &controller_base::controller_commands_callback,
                             this);
  ai_controller_commands_sub_ = nh_.subscribe("ai_controller_commands", 10, &controller_base::ai_controller_commands_callback, this);

  memset(&vehicle_state_, 0, sizeof(vehicle_state_));
  memset(&controller_commands_, 0, sizeof(controller_commands_));

  accept_ai_commands_ = false;

  nh_private_.param<double>("TRIM_E", params_.trim_e, 0.0);
  nh_private_.param<double>("TRIM_A", params_.trim_a, 0.0);
  nh_private_.param<double>("TRIM_R", params_.trim_r, 0.0);
  nh_private_.param<double>("TRIM_T", params_.trim_t, 0.0);
  nh_private_.param<double>("PWM_RAD_E", params_.pwm_rad_e, 1.0);
  nh_private_.param<double>("PWM_RAD_A", params_.pwm_rad_a, 1.0);
  nh_private_.param<double>("PWM_RAD_R", params_.pwm_rad_r, 1.0);
  nh_private_.param<double>("ALT_TOZ", params_.alt_toz, 20.0);
  nh_private_.param<double>("ALT_HZ", params_.alt_hz, 15.0);
  nh_private_.param<double>("TAU", params_.tau, 5.0);
  nh_private_.param<double>("COURSE_KP", params_.c_kp, 0.6);
  nh_private_.param<double>("COURSE_KD", params_.c_kd, -0.1);
  nh_private_.param<double>("COURSE_KI", params_.c_ki, 0.0);
  nh_private_.param<double>("ROLL_KP", params_.r_kp, 2.5); // 10.0 for aggressive
  nh_private_.param<double>("ROLL_KD", params_.r_kd, -0.92); // -0.92 for aggressive
  nh_private_.param<double>("ROLL_KI", params_.r_ki, 0.0);//0.10f);
  nh_private_.param<double>("PITCH_KP", params_.p_kp, 10.0);
  nh_private_.param<double>("PITCH_KD", params_.p_kd, -2.0);
  nh_private_.param<double>("PITCH_KI", params_.p_ki, 5.0);
  nh_private_.param<double>("PITCH_FF", params_.p_ff, 0.0);
  nh_private_.param<double>("AS_PITCH_KP", params_.a_p_kp, 0.05);
  nh_private_.param<double>("AS_PITCH_KD", params_.a_p_kd, -0.07);
  nh_private_.param<double>("AS_PITCH_KI", params_.a_p_ki, 0.0);
  nh_private_.param<double>("AS_THR_KP", params_.a_t_kp, 0.2);
  nh_private_.param<double>("AS_THR_KD", params_.a_t_kd, -0.01);
  nh_private_.param<double>("AS_THR_KI", params_.a_t_ki, 0.02);
  nh_private_.param<double>("AS_THR_FF", params_.a_t_ff, 0.05);
  nh_private_.param<double>("ALT_KP", params_.a_kp, 0.03);
  nh_private_.param<double>("ALT_KD", params_.a_kd, -0.00005);
  nh_private_.param<double>("ALT_KI", params_.a_ki, 0.003);
  nh_private_.param<double>("BETA_KP", params_.b_kp, 0.8);
  nh_private_.param<double>("BETA_KD", params_.b_kd, -2.5);
  nh_private_.param<double>("BETA_KI", params_.b_ki, 0.15);
  nh_private_.param<double>("MAX_E", params_.max_e, 0.610);
  nh_private_.param<double>("MAX_A", params_.max_a, 0.523);
  nh_private_.param<double>("MAX_R", params_.max_r, 1.0);
  nh_private_.param<double>("MAX_T", params_.max_t, 1.0);

  nh_private_.param<double>("TAKEOFF_KP", params_.t_kp, 5.0);
  nh_private_.param<double>("TAKEOFF_KD", params_.t_kd, -0.1);
  nh_private_.param<double>("TAKEOFF_KI", params_.t_ki, 0.0);
  nh_private_.param<double>("TAKEOFF_R_FF", params_.t_r_ff, 0.1);
  nh_private_.param<double>("VF_CHI_INFI", params_.chi_infi, 1.5708);
  nh_private_.param<double>("TAKEOFF_KPATH", params_.k_path, 1.0);

  nh_private_.param<double>("VH_KP", params_.vh_kp, 0.001);          // params for Vertical rate hold
  nh_private_.param<double>("VH_KD", params_.vh_kd, 0.0);
  nh_private_.param<double>("VH_KI", params_.vh_ki, 0.0);

  nh_private_.param<int>("ANGLE_IN_DEG", angle_in_deg_, 1);
  nh_private_.param<int>("AI_MODE", params_.ai_mode, 1);        // switches to AI mode after TAKEOFF -> Controller Commands will be taken from AI and not path manager or path follower
  // ROS_INFO("ai_mode %d", params_.ai_mode);

  func_ = boost::bind(&controller_base::reconfigure_callback, this, _1, _2);
  server_.setCallback(func_);

  actuators_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 10);
  internals_pub_ = nh_.advertise<rosplane_msgs::Controller_Internals>("controller_inners", 10);
  commanded_values_pub_ = nh_.advertise<rosplane_msgs::Commanded_Values>("commanded_values", 10);
  act_pub_timer_ = nh_.createTimer(ros::Duration(1.0/100.0), &controller_base::actuator_controls_publish, this);

  command_recieved_ = false;

  chi_0 = -360.0; // initialize chi_0 to an absurd value; chi will be given in radians
}

void controller_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
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
  if(chi_0 < -M_PI_F) chi_0 = vehicle_state_.chi; // since the aircraft is on the ground; chi = psi
}

void controller_base::controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr &msg)
{
  command_recieved_ = true;
  if(!accept_ai_commands_) {
    controller_commands_ = *msg;
    // ROS_INFO("Accepting rosplane commands");
  }
  if(angle_in_deg_) {
    controller_commands_.chi_c = controller_commands_.chi_c * DEG_2_RAD;
  }
  // ROS_INFO("Recieved Controller Commands : chi_c %f", controller_commands_.chi_c);
}

void controller_base::ai_controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr &msg)
{
  command_recieved_ = true;
  if(accept_ai_commands_) {
    ROS_INFO("Accepting AI commands %d",accept_ai_commands_);
    controller_commands_ = *msg;
  }
  if(angle_in_deg_) {
    controller_commands_.chi_c = controller_commands_.chi_c * DEG_2_RAD;
    // controller_commands_.phi_c = controller_commands_.phi_c * DEG_2_RAD;
  }
  // ROS_INFO("Recieved Controller Commands : chi_c %f", controller_commands_.chi_c);
}

void controller_base::reconfigure_callback(rosplane::ControllerConfig &config, uint32_t level)
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
  params_.a_t_ff = config.AS_THR_FF;

  params_.a_kp = config.ALT_KP;
  params_.a_kd = config.ALT_KD;
  params_.a_ki = config.ALT_KI;

  params_.b_kp = config.BETA_KP;
  params_.b_kd = config.BETA_KD;
  params_.b_ki = config.BETA_KI;

  params_.t_kp = config.TAKEOFF_KP;
  params_.t_kd = config.TAKEOFF_KD;
  params_.t_ki = config.TAKEOFF_KI;

  params_.chi_infi = config.TAKEOFF_VF_CHI_INFI;
  params_.k_path = config.TAKEOFF_KPATH;

  params_.vh_kp = config.VH_KP;
  params_.vh_kd = config.VH_KD;
  params_.vh_ki = config.VH_KI;

  ROS_INFO("DYNAMIC RECONFIGURE");
}

void controller_base::convert_to_pwm(controller_base::output_s &output)
{
  output.delta_e = output.delta_e*params_.pwm_rad_e;
  output.delta_a = output.delta_a*params_.pwm_rad_a;
  output.delta_r = output.delta_r*params_.pwm_rad_r;
}

void controller_base::actuator_controls_publish(const ros::TimerEvent &)
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
  input.Va_c = controller_commands_.Va_c;
  input.h_c = controller_commands_.h_c;
  input.chi_c = controller_commands_.chi_c;
  input.phi_ff = controller_commands_.phi_ff;
  input.Ts = 0.01f;
  input.beta = vehicle_state_.beta;
  input.vh_c = controller_commands_.vh_c / MS_TO_FPM;
 
  input.vh = vehicle_state_.vh / MS_TO_FPM;        // fpm to m/s
  input.land = controller_commands_.land;

  if (accept_ai_commands_){
    input.phi_c = controller_commands_.phi_c;
  }

  if(input.land) {
    ROS_INFO("CONTROLLER : LANDING MODE");
    // Make roll hold more agressive for landing
    params_.r_kp = 2.5;
    params_.r_kd = -0.9; 
  }
  else {
    // ROS_INFO("CONTROLLER : NOT LANDING MODE");
  }

  // Values required for proper takeoff; pn and pe can be used for cross-track error
  input.psi = vehicle_state_.psi;      
  input.pn = vehicle_state_.position[0];
  input.pe = vehicle_state_.position[1];

  struct output_s output;
  if (command_recieved_ == true)
  {
    control(params_, input, output);

    convert_to_pwm(output);

    rosflight_msgs::Command actuators;
    /* publish actuator controls */

    actuators.ignore = 0;
    actuators.header.stamp = ros::Time::now(); // Add this time stamp so that visualizations are possible
    actuators.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
    actuators.x = output.delta_a;//(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    actuators.y = output.delta_e;//(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    actuators.z = output.delta_r;//(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    actuators.F = output.delta_t;//(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

    /* Publish commanded values for the purpose of visualization*/
    commanded_values_.phi_c = output.phi_c;
    commanded_values_.theta_c = output.theta_c;
    commanded_values_.Va_c = input.Va_c;
    commanded_values_.h_c = -input.h_c;                 // Easier to understand since position is in NED
    commanded_values_.chi_c = controller_commands_.chi_c;
    commanded_values_.psi_c = output.psi_c;
    commanded_values_.chi_0 = chi_0;
    commanded_values_.vh_c = controller_commands_.vh_c;

    if(angle_in_deg_) {
      commanded_values_.phi_c = commanded_values_.phi_c * 1/DEG_2_RAD;
      commanded_values_.theta_c = commanded_values_.theta_c * 1/DEG_2_RAD;
      commanded_values_.chi_c = commanded_values_.chi_c * 1/DEG_2_RAD;
      commanded_values_.psi_c = commanded_values_.psi_c * 1/DEG_2_RAD;
      commanded_values_.chi_0 = commanded_values_.chi_0 * 1/DEG_2_RAD;
    }

    commanded_values_pub_.publish(commanded_values_);
    actuators_pub_.publish(actuators);

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
  ros::init(argc, argv, "rosplane_controller");
  rosplane::controller_base *cont = new rosplane::controller_example();

  ros::spin();

  return 0;
}

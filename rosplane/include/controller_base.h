/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Controller_Commands.h>
#include <rosplane_msgs/Controller_Internals.h>
#include <rosplane_msgs/Commanded_Values.h>

#include <dynamic_reconfigure/server.h>
#include <rosplane/ControllerConfig.h>

#define M_PI_F 3.14159265358979323846f
#define M_PI_2_F 1.57079632679489661923f

namespace rosplane
{

enum class alt_zones
{
  TAKE_OFF,
  CLIMB,
  DESCEND,
  ALTITUDE_HOLD, 
  LAND,                  // LAND mode makes use of the vertical_rate() for a constant rate of descent close to runway
  AI_MODE                // AI_MODE : General control loop to execute commands by other networks
};

class controller_base
{
public:
  controller_base();
  float spin();

protected:
  bool accept_ai_commands_; // bool to switch between rosplane takeoff -> ai mode -> landing mode (To be implemented)

  struct input_s
  {
    float Ts;               /** time step */
    float h;                /** altitude */
    float va;               /** airspeed */
    float phi;              /** roll angle */
    float theta;            /** pitch angle */
    float chi;              /** course angle */
    float p;                /** body frame roll rate */
    float q;                /** body frame pitch rate */
    float r;                /** body frame yaw rate */
    float Va_c;             /** commanded airspeed (m/s) */
    float h_c;              /** commanded altitude (m) */
    float chi_c;            /** commanded course (rad) */
    float phi_ff;           /** feed forward term for orbits (rad) */
    float beta;             /** side slip angle (rad) */

    float psi;              /** Want to follow correct heading during takeoff */
    float pn;               /* pn and pe will be handy for calculating crosstrack error during takeoff */
    float pe;
    float vh_c;             /*commanded vertical velocity (up is +ve) */
    float vh;               /** vertical velocity ; up is positive*/
    float phi_c;            /** commanded roll value used during AI mode*/
    bool land;              /* go into landing maneuver */
  };

  struct output_s
  {
    float theta_c;
    float delta_e;
    float phi_c;
    float delta_a;
    float delta_r;
    float delta_t;
    float psi_c;           /* Heading angle commanded during takeoff*/ 
    alt_zones current_zone;
  };

  struct params_s
  {
    double alt_hz;           /**< altitude hold zone */
    double alt_toz;          /**< altitude takeoff zone */
    double chi_infi;         /**Vector field param to give commanded heading based on cross track error during takeoff */
    double k_path;           /** Weight factor on error for holding yaw/course during takeoff */
    double tau;
    double c_kp;
    double c_kd;
    double c_ki;
    double r_kp;
    double r_kd;
    double r_ki;
    double p_kp;
    double p_kd;
    double p_ki;
    double p_ff;
    double a_p_kp;
    double a_p_kd;
    double a_p_ki;
    double a_t_kp;
    double a_t_kd;
    double a_t_ki;
    double a_t_ff;   // feedforward term for airspeed hold with throttle
    double a_kp;
    double a_kd;
    double a_ki;
    double b_kp;
    double b_kd;
    double b_ki;
    double t_kp;     // P control for takeoff
    double t_kd;     // D control for takeoff
    double t_ki;     // I control for takeoff
    double t_r_ff;   // Feed forward turn for rudder 
    double trim_e;
    double trim_a;
    double trim_r;
    double trim_t;
    double max_e;
    double max_a;
    double max_r;
    double max_t;
    double pwm_rad_e;
    double pwm_rad_a;
    double pwm_rad_r;
    double vh_kp; // PID params for vertical velocity 
    double vh_kd; // PID params for vertical velocity
    double vh_ki; // PID paramns for certical velocity
    int ai_mode; // True if we want to switch to AI after takeoff
  };

  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;
  float chi_0;                    /* initial heading to account for runway and calculate cross track error*/
  // rosplane_msgs::State vehicle_state_; 

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber controller_commands_sub_;
  ros::Subscriber ai_controller_commands_sub_;
  ros::Publisher actuators_pub_;
  ros::Publisher internals_pub_;
  ros::Timer act_pub_timer_;
  ros::Publisher commanded_values_pub_;

  struct params_s params_;            /**< params */
  rosplane_msgs::Controller_Commands controller_commands_;
  rosplane_msgs::State vehicle_state_;
  rosplane_msgs::Commanded_Values commanded_values_;

  void vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg);
  void controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr &msg);
  void ai_controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr &msg);
  bool command_recieved_;
  int angle_in_deg_; // 1 means angle will be in degrees and thus we would need to add a codeblock in callcack to convert angles

  dynamic_reconfigure::Server<rosplane::ControllerConfig> server_;
  dynamic_reconfigure::Server<rosplane::ControllerConfig>::CallbackType func_;

  void reconfigure_callback(rosplane::ControllerConfig &config, uint32_t level);

  /**
    * Convert from deflection angle to pwm
    */
  void convert_to_pwm(struct output_s &output);

  /**
    * Publish the outputs
    */
  void actuator_controls_publish(const ros::TimerEvent &);
};
} //end namespace

#endif // CONTROLLER_BASE_H

#include "path_manager_base.h"
#include "path_manager_example.h"

#define M_PI_F 3.14159265358979323846f
float DEG_2_RAD = M_PI_F/180.0;

namespace rosplane
{

path_manager_base::path_manager_base():
  nh_(ros::NodeHandle()), /** nh_ stuff added here */
  nh_private_(ros::NodeHandle("~"))
{
  nh_private_.param<double>("R_min", params_.R_min, 450);
  nh_private_.param<double>("update_rate", update_rate_, 10.0);
  nh_private_.param<int>("ANGLE_IN_DEG", angle_in_deg_, 1);

  vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
  new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);
  current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path", 10);

  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_manager_base::current_path_publish, this);

  num_waypoints_ = 0;

  state_init_ = false;

  landing_ = false;
}

void path_manager_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
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

  state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const rosplane_msgs::Waypoint &msg)
{
  // ROS_INFO("RECEIVED POINT %d , WITH VELOCITY %f",num_waypoints_, msg.Va_d);
  if (msg.clear_wp_list == true)
  {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    return;
  }
  if (msg.set_current || num_waypoints_ == 0)
  {
    waypoint_s currentwp;
    currentwp.w[0] = vehicle_state_.position[0];
    currentwp.w[1] = vehicle_state_.position[1];
    currentwp.w[2] = (vehicle_state_.position[2] > -25 ? msg.w[2] : vehicle_state_.position[2]);
    currentwp.chi_d = vehicle_state_.chi;
    currentwp.chi_valid = msg.chi_valid;
    currentwp.Va_d = msg.Va_d;

    waypoints_.clear();
    waypoints_.push_back(currentwp);
    num_waypoints_ = 1;
    idx_a_ = 0;
  }
  waypoint_s nextwp;
  nextwp.w[0]         = msg.w[0];
  nextwp.w[1]         = msg.w[1];
  nextwp.w[2]         = msg.w[2];
  nextwp.chi_d        = msg.chi_d;
  nextwp.chi_valid    = msg.chi_valid;
  nextwp.Va_d         = msg.Va_d;
  waypoints_.push_back(nextwp);
  num_waypoints_++;
  // ROS_INFO("Received waypoint : %d", num_waypoints_);
}

void path_manager_base::current_path_publish(const ros::TimerEvent &)
{

  struct input_s input;
  input.pn = vehicle_state_.position[0];               /** position north */
  input.pe = vehicle_state_.position[1];               /** position east */
  input.h =  -vehicle_state_.position[2];                /** altitude */
  input.chi = vehicle_state_.chi;

  struct output_s output;

  if (state_init_ == true)
  {
    manage(params_, input, output);
  }

  rosplane_msgs::Current_Path current_path;

  if (output.flag)
    current_path.path_type = current_path.LINE_PATH;
  else
    current_path.path_type = current_path.ORBIT_PATH;
  current_path.Va_d = output.Va_d;
  for (int i = 0; i < 3; i++)
  {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  // ROS_INFO("HEIGHT %f", output.c[2]);
  current_path.rho = output.rho;
  current_path.lambda = output.lambda;
  current_path.land = landing_;

  current_path_pub_.publish(current_path);
  // ROS_INFO("Path Manager : published current path");
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_path_manager");
  rosplane::path_manager_base *est = new rosplane::path_manager_example();

  ros::spin();

  return 0;
}

#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>

#define num_waypoints 5
float CHI_0 = -1.889;

void RotZ(float chi_0, float &pn, float &pe, float &chi) {
  float pn1 = cosf(chi_0)*pn - sinf(chi_0)*pe;
  float pe1 = sinf(chi+0)*pn + cosf(chi_0)*pe;
  chi += chi_0;
  pn = pn1;
  pe = pe1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_simple_path_planner");

  ros::NodeHandle nh_;
  ros::Publisher waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 10);

  float Va = 40;
  // float wps[5*num_waypoints] =
  // {
  //   200, 0, -50, 45*M_PI/180, Va,
  //   0, 200, -50, 45*M_PI/180, Va,
  //   200, 200, -50, 225*M_PI/180, Va,
  // };

  // float wps[5*num_waypoints] =
  // {
  //   0, -3000, -200, -15*M_PI/180, Va,
  //   3000, -3000, -200, 45*M_PI/180, Va,
  //   3000, 0, -200, 135*M_PI/180, Va,
  // };

  // float wps[5*num_waypoints] = 
  // {
  //   3000, 0, -200, -90*M_PI/180, Va,
  //   3000, -3000, -200, -180*M_PI/180, Va,
  //   -3000, -3000, -200, 90*M_PI/180, Va,
  //   -3000, 0, -200, 0*M_PI/180, Va,
  //   0,     0,  -100, 0*M_PI/180, 50
  // };
  // float wps[5*num_waypoints] = 
  // {
  //   -938.58, -2849.4, -200, -80*M_PI/180 + CHI_0, Va,
  //   -3787.97, -1910.81, -200, -170*M_PI/180 + CHI_0, Va,
  //   -1901.81, 3787.97, -200, 100*M_PI/180 + CHI_0, Va,
  //   938.58, 2849.39, -200, 10*M_PI/180 + CHI_0, Va
  //   // 0,     0,  -100, 0*M_PI/180 + CHI_0, 50
  // };

 float wps[5*num_waypoints] = 
  {
    -2162, -5604, -200, -80*M_PI/180 + CHI_0, Va,
    -6626, -4134.81, -200, -170*M_PI/180 + CHI_0, Va,
    -2871, 7263, -200, 100*M_PI/180 + CHI_0, Va,
    1877, 5698, -200, 10*M_PI/180 + CHI_0, Va,
    0,     0,  -5, 0*M_PI/180 + CHI_0, 20.0
  };

  // course heading is in NED frame

  for (int i(0); i < num_waypoints; i++)
  {
    ros::Duration(0.5).sleep();

    rosplane_msgs::Waypoint new_waypoint;
    //RotZ(CHI_0, wps[i*5 + 0], wps[i*5 + 1], wps[i*5 +3]);
    new_waypoint.w[0] = wps[i*5 + 0];
    new_waypoint.w[1] = wps[i*5 + 1];
    new_waypoint.w[2] = wps[i*5 + 2];
    new_waypoint.chi_d = wps[i*5 + 3];

    new_waypoint.chi_valid = true;
    new_waypoint.Va_d = wps[i*5 + 4];
    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    waypointPublisher.publish(new_waypoint);
  }
  ros::Duration(1.5).sleep();

  return 0;
}

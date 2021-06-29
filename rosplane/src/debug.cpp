#include "debug.hpp"

namespace rosplane {

debug::debug() : nh_(), nh_private_("~") {
    vehicle_state_sub_ = nh_.subscribe("/fixedwing/xplane/state", 10, &debug::vehicle_state_callback, this);
}

void debug::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
{
    vehicle_state_ = *msg;
    if (msg != nullptr) {
        std::cout<<vehicle_state_.position[0]<<" "<<vehicle_state_.position[1]<<" "<<vehicle_state_.position[2]<<"\n";
    }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_debugger");
  rosplane::debug *cont = new rosplane::debug();

  ros::spin();

  return 0;
}
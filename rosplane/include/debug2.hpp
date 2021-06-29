#include <ros/ros.h>
#include <rosplane_msgs/State.h>

namespace rosplane {

class debug {
public:
    debug();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber vehicle_state_sub_;
    rosplane_msgs::State vehicle_state_;
    void vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg);
};
}
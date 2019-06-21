#include "ros/ros.h"
#include "atwork_refbox_msgs/robot_state.h"

void clbk(const atwork_refbox_msgs::robot_state::ConstPtr& msg) {
    ROS_INFO("%d", msg->another_field);
    ROS_INFO("first point Position of Robot: x=%.2f, y=%.2f z=%.2f", msg->points[0].x, msg->points[0].y,msg->points[0].z);
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "my_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("my_topic", 1, clbk);

  ros::spin();

}


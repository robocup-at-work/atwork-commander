#include "ros/ros.h"
#include "atwork_refbox_ros_msgs/RobotState.h"

void clbk(const atwork_refbox_ros_msgs::RobotState::ConstPtr& msg) {
    //ROS_INFO("%d", msg->another_field);
    ROS_INFO("first point Position of Robot: x=%.2f, y=%.2f z=%.2f", msg->pose.x, msg->pose.y,msg->pose.theta);
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "my_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("my_topic", 1, clbk);

  ros::spin();

}

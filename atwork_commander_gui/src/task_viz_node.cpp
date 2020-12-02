#include "task_viz.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_viz");
  ROS_INFO("Task Viz starts...");  
  
  atwork_commander::TaskVirtualization TaskVirtualization;
  
  ros::spin();
  
  return 0;
}

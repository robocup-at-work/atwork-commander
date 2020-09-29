#include "task_viz.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_viz");
  ROS_INFO("Task Viz starts...");  
  
  atwork_commander::TaskVisualization TaskVisualization;
  
  ros::spin();
  
  return 0;
}

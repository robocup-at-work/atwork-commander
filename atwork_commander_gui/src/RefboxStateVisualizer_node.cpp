#include "RefboxStateVisualizer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "refbox_viz");
  
  atwork_commander::RefboxStateVisualizer viz;
  
  ros::spin();
  
  return 0;
}

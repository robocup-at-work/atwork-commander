#include <ros/ros.h>

#include <atwork_commander/Control.hpp>

#include <string>

using namespace std;
using namespace atwork_commander;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_setup");

  string taskName;
  if( !ros::param::get("~task", taskName) ) {
      ROS_ERROR_STREAM("[TEST-SETUP] No task name specified! Exiting");
      return -1;
  }

  Control control;


  try{
    while(control.state().state != Control::RefboxState::IDLE)
      ros::spinOnce();
    
    control.generate(taskName);
    
    while(control.state().state != Control::RefboxState::READY)
      ros::spinOnce();
    
    control.start();
    
    while(control.state().state != Control::RefboxState::PREPARATION)
      ros::spinOnce();

    control.forward();
  } catch(const ControlError& e) {
    ROS_ERROR_STREAM("[TEST-Setup] Setup error occurred: " << e.what());
    return -1;
  }

  return 0;
}

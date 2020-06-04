#include <ros/ros.h>

#include <string>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_setup");

  string refbox, taskName;
  if( !ros::param::get("~refbox", refbox) ) {
      refbox = "atwork_commander";
      ROS_WARN_STREAM("[TEST-SETUP] No Refbox name specified using \"" << refbox << "\"!");
  }
  if( !ros::param::get("~task", taskName) ) {
      ROS_ERROR_STREAM("[TEST-SETUP] No task name specified! Exiting");
      return -1;
  }

  Control control(refbox);

  if( !control.generate(taskName) )
    ROS_ERROR_STREAM("[TEST-Setup] Generation of Task " << taskName << "failed!");
  if( !control.start() )
    ROS_ERROR_STREAM("Generation of Task " << taskName << "failed!");
  control.forward();

  ROS_ERROR_STREAM("[TEST_SETUP] Not implemented yet");
  return 0;
}

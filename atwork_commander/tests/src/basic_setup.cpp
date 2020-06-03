#include <ros/ros.h>

#include <string>

using namespace std;

string gTaskName;
string gRefboxName;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_setup");

  if( !ros::param::get("~refbox", gRefboxName) ) {
      gRefboxName = "atwork_commander";
      ROS_WARN_STREAM_NAMED("setup", "[TEST-SETUP] No Refbox name specified using \"" << gRefboxName << "\"!");
  }
  if( !ros::param::get("~task", gTaskName) ) {
      ROS_ERROR_STREAM_NAMED("setup", "[TEST-SETUP] No task name specified! Exiting");
      return -1;
  }

  ROS_ERROR_STREAM("[TEST_SETUP] Not implemented yet");
  return 0;
}

#include "JurekGen.cpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <gtest/gtest.h>

#include <thread>

using namespace std;
using namespace chrono;

static void activateDebug() {
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
    this_thread::sleep_for(seconds(1)); // Necessary to allow rosconsole to react to logger level change
    ROS_DEBUG_STREAM("Debugging mode activated!");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_task_generator");
  ::testing::InitGoogleTest(&argc, argv);

  activateDebug();
  thread run([](){
    ROS_DEBUG_STREAM("ROS spin thread initalized!");
    ros::spin();
  });


  int result = RUN_ALL_TESTS();  
  ros::shutdown();
  run.join();
  return result;
}

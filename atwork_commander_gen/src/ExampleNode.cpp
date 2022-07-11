#include <atwork_commander_gen/TaskGenerator.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <exception>
#include <chrono>
#include <thread>

using namespace atwork_commander;
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
  activateDebug();

  string taskName;
  if( !ros::param::get("taskName", taskName)) {
    ROS_ERROR_STREAM("No taskName specified!");
    return -1;
  }

  int result = 0;
  thread run(
    [taskName, &result](){
      TaskGenerator gen("arena", "tasks", "plugin");
      ROS_INFO_STREAM("Arena Config:" << endl << gen.config().arena());
      ROS_INFO_STREAM("Tasks Config: " << endl << gen.config().tasks());
      auto task = gen(taskName);
      ROS_INFO_STREAM("Example task:" << endl << task);
      gen.check(task);
      ROS_INFO_STREAM("Example task correct");
      ros::shutdown();
    }
  );

  while(ros::ok())
    ros::spin();

  run.join();

  return result;
}

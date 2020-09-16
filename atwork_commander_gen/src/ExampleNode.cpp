#include <atwork_commander_gen/TaskGenerator.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <exception>
#include <thread>
#include <chrono>

using namespace atwork_commander;
using namespace std;
using namespace chrono;

static void activateDebug() {
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
    this_thread::sleep_for(seconds(1)); // Necessary to allow rosconsole to react to logger level change
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_task_generator");
  activateDebug();

  try {
    TaskGenerator gen("arena", "tasks", "plugin");
    cout << "Arena Description:" << endl << gen.config().arena() << endl;
    cout << "Task Definitions:" << endl << gen.config().tasks() << endl;
    auto task = gen(string("example"));
    cout << "Tasks:" << endl << task << endl;
  }
  catch(const exception& e) {
    cerr << "Exception occured: \n" << e.what() << endl;
    return -1;
  }

  return 0;
}

#include <atwork_refbox_task_generator/TaskGenerator.h>

using namespace atwork_refbox_ros;
using namespace std;

TaskGenerator::TaskGenerator(Options globalOptions, Tasks tasks, Workstations workstations, Waypoints waypoints) 
  : mTasks(tasks), mGlobals(globalOptions), mWorkstations(workstations), mWaypoints(waypoints)
{

}

const Task& TaskGenerator::operator()(string name) {
  Task task;
  mLastTask = task;
  return mLastTask;
}

#include <atwork_refbox_ros_task_generator/TaskGenerator.h>


using namespace atwork_refbox_ros;
using namespace std;

TaskGenerator::TaskGenerator(Options globalOptions, TaskDefinitions tasks, Workstations workstations) 
  : mNode(globalOptions, tasks, workstations)
{

}

const Task& TaskGenerator::operator()(string name) {
  Task task;
  mLastTask = task;
  return mLastTask;
}

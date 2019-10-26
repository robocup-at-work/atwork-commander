#pragma once

#include <map>
#include <string>
#include <atwork_refbox_ros_msgs/Task.h>

namespace atwork_refbox_ros {

using Task = atwork_refbox_ros_msgs::Task;

struct Workstation {

};

struct Waypoint {

};

using TaskDefinition = std::map<std::string, int>;
using TaskDefinitions = std::map<std::string, TaskDefinition>;
using Options = std::map<std::string, int>;
using Workstations = std::map<std::string, Workstation>;
using Waypoints = std::map<std::string, Waypoint>;

class TaskGenerator {
  private:
    TaskDefinitions mTasks;
    Options mGlobals;
    Workstations mWorkstations;
    Waypoints mWaypoints;
    Task mLastTask;
  public:
    TaskGenerator(Options globalOptions, TaskDefinitions tasks, Workstations workstations, Waypoints waypoints);
    const Task& operator()(std::string taskName);
};

}

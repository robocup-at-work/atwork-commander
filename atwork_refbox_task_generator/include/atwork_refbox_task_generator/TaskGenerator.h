#pragma once

#include <map>
#include <string>

namespace atwork_refbox_ros {

struct Workstation {

};

struct Waypoint {

};

using Task = std::map<std::string, int>;
using Tasks = std::map<std::string, Task>;
using Options = std::map<std::string, int>;
using Workstations = std::map<std::string, Workstation>;
using Waypoints = std::map<std::string, Waypoint>;

class TaskGenerator {
  private:
    Tasks mTasks;
    Options mGlobals;
    Workstations mWorkstations;
    Waypoints mWaypoints;
    Task mLastTask;
  public:
    TaskGenerator(Options globalOptions, Tasks tasks, Workstations workstations, Waypoints waypoints);
    const Task& operator()(std::string taskName);
};

}

#pragma once

#include <atwork_commander_msgs/Task.h>

#include <string>

namespace atwork_commander {
namespace task_generator {

class PluginInterface {
  public:
    using Task = ::atwork_commander_msgs::Task;
    virtual ~PluginInterface() {}
    virtual void onInit(const std::string& arenaConfig, const std::string& taskConfig);
    virtual Task generate(const std::string& taskName) = 0;
    virtual bool check(const Task& task) const = 0;
};

}
}

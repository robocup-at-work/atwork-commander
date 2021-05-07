#pragma once

#include <atwork_commander_msgs/Task.h>

#include <string>

namespace atwork_commander {

class ConfigParserInterface;

namespace task_generator {


class GeneratorPluginInterface {
  public:
    using Task = ::atwork_commander_msgs::Task;
    virtual ~GeneratorPluginInterface() {}
    virtual void onInit(const std::string& arenaConfig, const std::string& taskConfig) = 0;
    virtual Task generate(const std::string& taskName) = 0;
    virtual bool check(const Task& task) const = 0;
    virtual ConfigParserInterface& config() = 0;
};

}
}

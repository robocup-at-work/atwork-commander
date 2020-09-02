#pragma once

#include <atwork_commander_msgs/Task.h>

#include <string>
#include <memory>

namespace atwork_commander {

namespace task_generator {
  class PluginInterface;
}

class TaskGenerator {
  private:
    using Interface = task_generator::PluginInterface;
    using InterfacePtr = std::shared_ptr<Interface>;
    InterfacePtr mImpl;

  public:
    using Task = atwork_commander_msgs::Task;

    TaskGenerator(const std::string& arenaConfig, const std::string& taskConfig, const std::string& pluginConfig);

    /** \brief Generate new randomized task based on task definition
     *
     *  \param taskName The name of the task as defined by the supplied TaskDefinitions
     *  \return Task instance ready to be transmitted
     *  \throw std::runtime_error if something goes wrong
     **/
    Task operator()(const std::string& taskName);

    /** \brief Check task for basic soundness
     *
     *  \param task The task to check
     *  \return true if sound, false otherwise
     *  \throw None
     **/
    bool check(const Task& task) const;
};

}

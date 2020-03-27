#pragma once

#include "Definitions.h"

#include <atwork_commander_msgs/Task.h>

namespace atwork_commander {

using Task = atwork_commander_msgs::Task;

class TaskGeneratorImpl;

class TaskGenerator {
  private:
    TaskGeneratorImpl* mImpl;
  public:
    TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks);
    ~TaskGenerator();

    /** \brief Generate new randomized task based on task definition
     *
     *  \param taskName The name of the task as defined by the supplied TaskDefinitions
     *  \return Task instance ready to be transmitted
     *  \throw std::runtime_error if something goes wrong
     **/
    Task operator()(std::string taskName);

    /** \brief Check task for basic soundness
     *
     *  \param task The task to check
     *  \return true if sound, false otherwise
     *  \throw None
     **/
    bool check(const Task& task) const;
};

}

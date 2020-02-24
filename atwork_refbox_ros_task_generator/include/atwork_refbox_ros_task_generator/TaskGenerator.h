#pragma once

#include "Definitions.h"

#include <atwork_refbox_ros_msgs/Task.h>

namespace atwork_refbox_ros {

using Task = atwork_refbox_ros_msgs::Task;

class TaskGeneratorImpl;

class TaskGenerator {
  private:
    TaskGeneratorImpl* mImpl;
  public:
    TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks);
    ~TaskGenerator();
    Task operator()(std::string taskName);
};

}

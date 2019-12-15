#pragma once

#include <map>
#include <string>
#include <atwork_refbox_ros_msgs/Task.h>
#include "Definitions.h"
#include "receiver_node_atc.h"

namespace atwork_refbox_ros {

using Task = atwork_refbox_ros_msgs::Task;

class TaskGenerator {
  private:
    TaskDefinitions mTasks;
    ReceiverNode mNode;
  public:
    TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks);
    Task operator()(std::string taskName);
};

}

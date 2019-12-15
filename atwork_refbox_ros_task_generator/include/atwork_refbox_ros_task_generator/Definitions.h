#pragma once

#include <unordered_map>
#include <string>

namespace atwork_refbox_ros {

struct ArenaDescription {
  std::map<std::string, bool> cavities;
  std::unordered_map<std::string, std::string> workstations;
};

using TaskDefinition = std::unordered_map<std::string, int>;
using TaskDefinitions = std::unordered_map<std::string, TaskDefinition>;

}

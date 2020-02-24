#pragma once

#include <unordered_map>
#include <map>
#include <string>
#include <iosfwd>

namespace atwork_refbox_ros {

struct ArenaDescription {
  std::map<std::string, bool> cavities;
  std::unordered_map<std::string, std::string> workstations;
};

using TaskDefinition = std::unordered_map<std::string, int>;
using TaskDefinitions = std::unordered_map<std::string, TaskDefinition>;

}

std::ostream& operator<<(std::ostream& os, const atwork_refbox_ros::ArenaDescription& arena);
std::ostream& operator<<(std::ostream& os, const atwork_refbox_ros::TaskDefinition& def);
std::ostream& operator<<(std::ostream& os, const atwork_refbox_ros::TaskDefinitions& defs);

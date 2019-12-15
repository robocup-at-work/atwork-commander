#pragma once

#include <unordered_map>
#include <string>

namespace atwork_refbox_ros {

using TaskDefinition = std::unordered_map<std::string, int>;
using TaskDefinitions = std::unordered_map<std::string, TaskDefinition>;
using Options = std::unordered_map<std::string, int>;
using Workstations = std::unordered_map<std::string, std::string>;

}

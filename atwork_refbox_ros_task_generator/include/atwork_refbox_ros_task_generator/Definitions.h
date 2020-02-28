#pragma once

#include <unordered_map>
#include <map>
#include <string>
#include <iosfwd>

namespace atwork_refbox_ros {

/** \brief Configuration Information for TaskGenerator representing Arena
 **/
struct ArenaDescription {
  std::map<std::string, bool> cavities; ///< Available Cavities for PPT
  std::unordered_map<std::string, std::string> workstations; ///< Mapping of available Workstations and Workstation Types
};

using TaskDefinition = std::unordered_map<std::string, int>; ///< Definition of parameters of a single Task
using TaskDefinitions = std::unordered_map<std::string, TaskDefinition>; ///< Definition of all available Tasks with the appropriate configuration parameters

}

/** \brief Output operator for ArenaDescriptions
 *  \param os Output Stream to output ArenaDescription
 *  \param arena the ArenaDescription to output
 *  \return the modified Output Stream
 * **/
std::ostream& operator<<(std::ostream& os, const atwork_refbox_ros::ArenaDescription& arena);

/** \brief Output operator for single Task Definitions
 *  \param os Output Stream to output the Task Definition
 *  \param arena the ArenaDescription to output
 *  \return the modified Output Stream
 * **/
std::ostream& operator<<(std::ostream& os, const atwork_refbox_ros::TaskDefinition& def);

/** \brief Output operator for all Task Definitions
 *  \param os Output Stream to output ArenaDescription
 *  \param arena the ArenaDescription to output
 *  \return the modified Output Stream
 * **/
std::ostream& operator<<(std::ostream& os, const atwork_refbox_ros::TaskDefinitions& defs);

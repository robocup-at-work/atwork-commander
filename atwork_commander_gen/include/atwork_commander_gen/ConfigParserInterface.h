#pragma once

#include <unordered_map>
#include <map>
#include <string>
#include <iosfwd>
#include <vector>

namespace atwork_commander {

using StringList = std::vector<std::string>;              ///< List of character strings
using ParameterType = std::unordered_map<std::string, int>;  ///< List of parameter key-value pairs

/** \brief Configuration Information for TaskGenerator representing Arena **/
struct ArenaDescription {
  StringList cavities;                                       ///< Available Cavities for PPT
  ParameterType objects;                                     ///< max. available count of each object type
  std::unordered_map<std::string, std::string> waypoints;    ///< additional waypoints with attached allows orientations ("NESW")
  std::unordered_map<std::string, std::string> workstations; ///< Mapping of available Workstations and Workstation Types
};

/** \brief Configuration Information for TaskGenerator representing specific task type **/
struct TaskDefinition {
  ParameterType objects;                                       ///< max. requested Object count
  StringList    cavities;                                      ///< requested deactivated Cavity types
  StringList    normalTableTypes { "00", "05", "10", "15" };   ///< types of normal tables
  StringList    allowedTables;                                 ///< Explicit list of allowed tables, ignored if empty
  StringList    ttTypes { "TT" };                              ///< type name of TurnTable
  StringList    ppTypes { "PP" };                              ///< type name of PrecisionPlacement
  StringList    shTypes { "SH" };                              ///< type name of Shelf
  ParameterType parameters {                                   ///< Various task specification parameters
    // Time
    { "prep_time", 1 },
    { "exec_time", 1 },
    // Nav
    { "waypoints", 0 },
    { "obstacles", 0 },
    { "barrier_tapes", 0 },
    // Transport
    { "objects", 1 },
    { "decoys", 0 },
    { "tables", 0 },
    { "ref_position", 0 },
    { "ref_rotation", 0 },
    { "ref_orientation", 0 },
    { "arbitrary_surfaces", 0},
    // Container
    { "container_placing_b", 0 },
    { "container_placing_r", 0 },
    { "paired_containers", 0 },
    { "container_on_shelf", 0 },
    { "container_on_tt", 0 },
    // PP
    { "pp_placing", 0 },
    { "ref_cavity_position", 0 },
    { "ref_cavity_rotation", 0 },
    { "ref_cavity_orientation", 0 },
    // RT
    { "tt_grasping", 0 },
    { "tt_placing", 0 },
    { "ref_tt_direction", 0 },
    // Shelf
    { "shelfes_grasping", 0 },
    { "shelfes_placing", 0 }
  };
};

/** Definition of all available Tasks with the appropriate configuration parameters **/
using TaskDefinitions = std::unordered_map<std::string, TaskDefinition>;

class ConfigParserInterface {
  private:
    std::string mTaskConfig;
    std::string mArenaConfig;
  protected:
    virtual void update() = 0;
  public:
    void reload(const std::string& arenaConfig, const std::string& taskConfig) {
      std::string oldTaskConfig = mTaskConfig;
      std::string oldArenaConfig = mArenaConfig;
      mTaskConfig = taskConfig;
      mArenaConfig = arenaConfig;
      try {
        update();
      } catch(const std::exception& e) {
        mTaskConfig = oldTaskConfig;
        mArenaConfig = oldArenaConfig;
        throw e;
      }
      mTaskConfig = taskConfig;
      mArenaConfig = arenaConfig;
    }
    const std::string& taskConfig() const { return mTaskConfig; }
    const std::string& arenaConfig() const { return mArenaConfig; }
    virtual const TaskDefinitions& tasks() const = 0;
    virtual const ArenaDescription& arena() const = 0;
};

}

/** \brief Output operator for ArenaDescriptions
 *  \param os Output Stream to output ArenaDescription
 *  \param arena the ArenaDescription to output
 *  \return the modified Output Stream
 **/
std::ostream& operator<<(std::ostream& os, const atwork_commander::ArenaDescription& arena);

/** \brief Output operator for single Task Definitions
 *  \param os Output Stream to output the Task Definition
 *  \param arena the ArenaDescription to output
 *  \return the modified Output Stream
 **/
std::ostream& operator<<(std::ostream& os, const atwork_commander::TaskDefinition& def);

/** \brief Output operator for all Task Definitions
 *  \param os Output Stream to output ArenaDescription
 *  \param arena the ArenaDescription to output
 *  \return the modified Output Stream
 **/
std::ostream& operator<<(std::ostream& os, const atwork_commander::TaskDefinitions& defs);

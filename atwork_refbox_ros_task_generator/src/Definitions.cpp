#include <atwork_refbox_ros_task_generator/Definitions.h>

#include <iostream>

using namespace atwork_refbox_ros;
using namespace std;

ostream& operator<<(ostream& os, const ArenaDescription& arena) {
  os << "Arena:\n\tAllowed Cavities: ";
  for(const auto& item: arena.cavities)
    if ( item.second ) os << item.first << " ";
  os << endl << "\tWorkstations: ";
  for(const auto& item: arena.workstations)
    os << item.first << "( " << item.second << ") ";
  return os;
}

ostream& operator<<(ostream& os, const TaskDefinition& def) {
  for(const auto& item : def)
    os << "\t" << item.first << " = " << item.second << endl;
  return os;
}

ostream& operator<<(ostream& os, const TaskDefinitions& defs) {
  os << "Task Definitions:" << endl;
  for(const auto& item : defs)
    os << item.first << endl << item.second;
  return os;
}

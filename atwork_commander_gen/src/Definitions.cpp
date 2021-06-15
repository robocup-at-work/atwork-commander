#include <atwork_commander_gen/ConfigParserInterface.h>
#include "Generator.h"
#include <iostream>

using namespace atwork_commander;
using namespace std;

ostream& operator<<(ostream& os, const ArenaDescription& arena) {
  os << "Arena:\n\tAllowed Cavities: [";
  for(const auto& item: arena.cavities)
    os << item << " ";
  os << endl << "]" << endl << "\tWorkstations:" << endl;
  for(const auto& item: arena.workstations)
    os << "\t\t" << item.first << "(" << item.second << ")" << endl;
  os << endl << "\tWaypoints:";
  for(const auto& item: arena.waypoints)
    os << "\t\t" << item.first << "(" << item.second << ")" << endl;
  os << endl << "\tObjects:" << endl;
  for(const auto& item: arena.objects)
    os << "\t\t" << item.first << "(" << item.second << ")" << endl;
  return os;
}

ostream& operator<<(ostream& os, const TaskDefinition& def) {
  os <<    "\tObjects        : [";
  for(const auto& item : def.objects)
    os << item.first << "( " << item.second << " ) ";
  os << "]\n\tNormal Tables  : [";
  for(const auto& item : def.normalTableTypes)
    os << item << " ";
  os << "]\n\tSpecial Tables: " << endl;
  os << "\t\tTurnTables: [";
  for( const auto& name: def.ttTypes )
    os << name << " ";
  os << "]" << endl << "\t\tPrecision Placement: [";
  for( const auto& name: def.ppTypes )
    os << name << " ";
  os << "]" << endl << "\t\tShelf: [";
  for( const auto& name: def.shTypes )
    os << name << " ";
  os << "]" << endl << "\tParameters:" << endl;
  for(const auto& item : def.parameters)
    os << "\t\t" << item.first << " = " << item.second << endl;
  return os;
}

ostream& operator<<(ostream& os, const TaskDefinitions& defs) {
  os << "Task Definitions:" << endl;
  for(const auto& item : defs)
    os << item.first << endl << item.second;
  return os;
}

unsigned int atwork_commander::task_generator::Object::globalID = 1;

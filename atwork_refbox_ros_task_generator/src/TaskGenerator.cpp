#include <atwork_refbox_ros_task_generator/TaskGenerator.h>

#include <ros/console.h>

#include <sstream>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <regex>

using namespace std;


namespace atwork_refbox_ros {

class Object;
class Table;

using ObjectPtr = Object*;
using TablePtr = Table*;

struct Object {
  const string& type;
  const unsigned int id;
  TablePtr source;
  TablePtr destination;
  ObjectPtr container;
  Object(const string& type, unsigned int id): type(type), id(id) {}
};

struct Table {
  std::string name ="";
  std::string type ="";
  std::vector<ObjectPtr> container;
  Table() = default;
  Table(const std::string& name, const std::string& type)
    : name(name), type(type) {}
  void reset() { container.clear(); }
};
}


ostream& operator<<(ostream& os, const atwork_refbox_ros::Table& t) {
  os << "Table " << t.name << "(" << t.type << "):";
  os << "\tContainer: [";
  for (const auto& c: t.container)
    os << c->type << "(" << c->id << ")" << " ";
  return os << "]";
}

ostream& operator<<(ostream& os, const atwork_refbox_ros::Object& o) {
  os << "Object " << o.type << "(" << o.id << "):";
  if ( o.source )      os << "\tSource     : " << o.source->name      << "(" << o.source->type      << ")";
  if ( o.destination ) os << "\tDestination: " << o.destination->name << "(" << o.destination->type << ")";
  if ( o.container )   os << "\tContainer  : " << o.container->type   << "(" << o.container->id     << ")";
  return os;
}

ostream& operator<<(ostream& os, const vector<atwork_refbox_ros::Object>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?"\n":"");
  return os;
}

ostream& operator<<(ostream& os, const vector<const string*>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << *v[i] << (i+1!=v.size()?" ":"");
  return os;
}

ostream& operator<<(ostream& os, const unordered_multimap<string, atwork_refbox_ros::Table>& m) {
  for (const auto& item: m)
    os << item.first << " = " << item.second << endl;
  return os;
}

enum class Orientation : unsigned int {
  VERTICAL,
  HORIZONTAL
};

namespace atwork_refbox_ros {
/**
 * \brief convert table description from arena format to internal format
 * \param arena Description of Arena
 * \return Vector of TaskGenerator::Table objects filled with arena information
 **/
static auto extractTables(const ArenaDescription& arena) {
  unordered_multimap<string, Table> tables;
  for (const auto& table: arena.workstations)
    tables.emplace(table.second, Table(table.first, table.second));
  return tables;
}

/**
 * \brief convert information on usable cavities of PPT to internal format
 * \param arena Description of Arena
 * \return Vector of string containing names of usable cavities
 **/
static auto extractCavities(const ArenaDescription& arena) {
  unordered_map<string, Orientation> cavities;
  for (const auto& item: arena.cavities) {

  }
  transform(arena.cavities.begin(), arena.cavities.end(), insert_iterator(cavities.begin()),
            [](const pair<string, bool>& item){ return item.second ? item.first : ""; });
  auto newEnd = remove_if(cavities.begin(), cavities.end(), [](const string& s){return s!="";});
  cavities.erase(newEnd, cavities.end());
  return cavities;
}

/** Task Generation Implementation
 *
 * Implements the generation of Task according to supplied configurations.
 *
 * 
 *
 **/
class TaskGeneratorImpl {

  TaskDefinitions mTasks;
  unordered_multimap<string, Table> mTables;
  vector<const string*> mObjectTypes;
  unordered_map<const string*, Orientation> mCavities;
  unsigned int mLastID = 0;

  Object nextObject(const string& type) { return Object(type, mLastID++); }

  void extractObjectTypes(const string& task) {
    mObjectTypes.resize(mTasks[task].size());
    auto it=mObjectTypes.begin();
    for (const auto& item: mTasks[task])
      if ( item.second && regex_match(item.first, regex("[A-Z0-9_]+")))
        *it++=&item.first;
    mObjectTypes.erase(it, mObjectTypes.end());
  }

  Task generate(const TaskDefinition& def){
    vector<Object> container;
    vector<Object> objects;
    for (auto& table: mTables)
      table.second.reset();
    mLastID = 0;

    

    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Objects:\n" << objects);
    // TODO

    return Task();
  }

  void sanityCheck(std::string task = "") {
    if (task == "") {
      if (mTables.size() < 2) throw runtime_error("At least two tables need to exist in the arena!");
      if (mTasks.empty()) throw runtime_error("No Tasks configured!");
      for (auto& task : mTasks ) {
        if ( task.second["object_count"] == 0 && task.second["waypoint_count"] == 0 ) {
          ostringstream os;
          os << task.first << ": Empty Task defined!";
          throw runtime_error(os.str());
        }
      }
      return;
    }
    if (mTasks[task]["object_count"]>=0 && mObjectTypes.empty() ) {
      ostringstream os;
      os << task << ": Transportation Task without allowed object defined!";
      throw runtime_error(os.str());
    }
    if (mTasks[task]["waypoint_count"]>=0 && mTables.size()<mTasks[task]["waypoint_count"] ) {
      ostringstream os;
      os << task << ": Navigation Task without enough workstations defined!";
      throw runtime_error(os.str());
    }
    if ( ( mTasks[task]["shelf_grasping"] || mTasks[task]["shelf_picking"] ) && mTables.count("SH")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving shelf requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( ( mTasks[task]["rt_grasping"] || mTasks[task]["rt_picking"] ) && mTables.count("TT")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving Rotating Table requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( mTasks[task]["pp"] && mTables.count("PP")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving Precision Placement requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( mTasks[task]["table_height_0"] && mTables.count("00")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving zero height table requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( mTasks[task]["table_height_5"] && mTables.count("05")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving 5cm table requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( mTasks[task]["table_height_10"] && mTables.count("10")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving 5cm table requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( mTasks[task]["table_height_15"] && mTables.count("15")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving 5cm table requested in Arena without it!";
      throw runtime_error(os.str());
    }
    //TODO
  }

  public:
    TaskGeneratorImpl(const ArenaDescription& arena, const TaskDefinitions& tasks)
      : mTasks(tasks), mTables(extractTables(arena)), mCavities(extractCavities(arena))
    {
      sanityCheck();
    }

    Task operator()(std::string taskName) {
      auto task = find_if(mTasks.begin(), mTasks.end(),
                         [taskName](const auto& item){ return item.first == taskName; }
                  );
      if (task == mTasks.end()) {
        ostringstream os;
        os << "No Task " << taskName << " configured. Valid tasks are: ";
        for (const auto& item: mTasks)
          os << item.first << " ";
        throw runtime_error(os.str());
      }
      extractObjectTypes(task->first);
      ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Tables:\n" << mTables);
      ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] ObjectTypes:\n" << mObjectTypes);
      ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Last ObjectID = " << mLastID);
      sanityCheck(task->first);
      return generate(task->second);
    }
};

TaskGenerator::TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks)
  : mImpl(new TaskGeneratorImpl(arena, tasks)) {}

TaskGenerator::~TaskGenerator() { delete mImpl; }

Task TaskGenerator::operator()(string taskName) { return mImpl->operator()(taskName); }

}

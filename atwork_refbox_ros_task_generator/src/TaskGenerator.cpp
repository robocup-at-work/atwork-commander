#include <atwork_refbox_ros_task_generator/TaskGenerator.h>

#include <ros/console.h>

#include <sstream>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <regex>
#include <iterator>

using namespace std;


namespace atwork_refbox_ros {

class Object;
class Table;

using ObjectPtr = Object*;
using TablePtr = Table*;

enum class Orientation : unsigned int {
  FREE,
  VERTICAL,
  HORIZONTAL
};

enum class Type : unsigned int {
  UNKNOWN,
  OBJECT,
  COLORED_OBJECT,
  CAVITY,
  CONTAINER
};

struct ObjectBase {
  static Type extractType(const string& typeName) {
    size_t len = typeName.length();
    char last = typeName[ len - 1 ];
    if (  last == 'H' || last == 'V' )
      return Type::CAVITY;
    if (  last == 'G' || last == 'B' )
      return Type::COLORED_OBJECT;
    if ( typeName.substr(0, sizeof("CONTAINER")) == "CONTAINER" )
      return Type::CONTAINER;
    return Type::OBJECT;
  }

  string extractForm(const string& typeName) const {
    switch ( type ) {
      case ( Type::CAVITY):
      case ( Type::COLORED_OBJECT): return typeName.substr( 0, typeName.length() - 2 );
      case ( Type::CONTAINER ): return "DEFAULT";
      default: return typeName;
    }
  }

  string extractColor(const string& typeName) const {
    switch ( type ) {
      case ( Type::COLORED_OBJECT): return typeName.substr( typeName.length() - 1 );
      case ( Type::CONTAINER ): return typeName.substr( sizeof("CONTAINER_") );
      default: return "DEFAULT";
    }
  }

  Orientation extractOrientation(const string& typeName) const {
    if ( type ==  Type::CAVITY)
      switch ( typeName[ typeName.length() - 1 ] ) {
        case ( 'H' ): return Orientation::HORIZONTAL;
        case ( 'V' ): return Orientation::VERTICAL;
      };
    return Orientation::FREE;
  }

  const Type type = Type::UNKNOWN;
  const string form = "";
  const string color = "";
  const unsigned int count = 0;
  Orientation orientation = Orientation::FREE;
  ObjectBase() = default;
  ObjectBase(const string typeName)
    : type( extractType( typeName ) ), form( extractForm( typeName ) ),
      color( extractColor( typeName ) ),
      orientation( extractOrientation( typeName ) )
  {}
};

struct ObjectType : public ObjectBase {
  unsigned int count = 0;
  ObjectType( const string& typeName, unsigned int count )
    : ObjectBase( typeName ), count(count)
  {}
  operator bool() const { return count; }
  ObjectType& operator--() {
    count--;
    return *this;
  }
  ObjectType operator--(int) {
    ObjectType temp(*this);
    count--;
    return temp;
  }
};

struct Object : public ObjectBase {
  static unsigned int globalID;
  const unsigned int id=globalID++;
  TablePtr source;
  TablePtr destination;
  ObjectPtr container;
  Object() = default;
  Object(const ObjectType& type): ObjectBase(type) {}
  static void reset() { globalID = 0; }
};

unsigned int Object::globalID = 0;

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

ostream& operator<<(ostream& os, const atwork_refbox_ros::Type type) {
  switch ( type ) {
    case ( atwork_refbox_ros::Type::CAVITY ):         return os << "Cavity";
    case ( atwork_refbox_ros::Type::CONTAINER ):      return os << "Container";
    case ( atwork_refbox_ros::Type::COLORED_OBJECT ): return os << "Colored Object";
    case ( atwork_refbox_ros::Type::OBJECT ):         return os << "Plain Object";
    default:                                          return os << "UNKNOWN";
  }
}

ostream& operator<<(ostream& os, const atwork_refbox_ros::Orientation o) {
  switch ( o ) {
    case( atwork_refbox_ros::Orientation::VERTICAL )  : return os << "V";
    case( atwork_refbox_ros::Orientation::HORIZONTAL ): return os << "H";
    default                        : return os << "UNKNOWN";
  }
}

ostream& operator<<(ostream& os, const atwork_refbox_ros::ObjectType& type) {
  switch ( type.type ) {
    case ( atwork_refbox_ros::Type::CAVITY ):         return os << type.form << "_" << type.orientation;
    case ( atwork_refbox_ros::Type::CONTAINER ):
    case ( atwork_refbox_ros::Type::COLORED_OBJECT ): return os << type.form << "_" << type.color;
    case ( atwork_refbox_ros::Type::OBJECT ):         return os << type.form;
    default:                                           return os << "UNKNOWN OBJECT TYPE";
  }
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

template<typename T>
ostream& operator<<(ostream& os, const vector<const T*>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << *v[i] << (i+1!=v.size()?" ":"");
  return os;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<T>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?" ":"");
  return os;
}

template<typename K, typename V>
ostream& operator<<(ostream& os, const unordered_multimap<K, V>& m) {
  for (const auto& item: m)
    os << item.first << " = " << item.second << endl;
  return os;
}

namespace atwork_refbox_ros {

/** Task Generation Implementation
 *
 * Implements the generation of Task according to supplied configurations.
 *
 * 
 *
 **/
class TaskGeneratorImpl {

  static auto extractCavities(const ArenaDescription& arena) {
    vector<ObjectType> cavities;
      for (const auto& item: arena.cavities)
        if ( item.second )
          cavities.emplace_back(item.first, item.second);
    return cavities;
  }

  TaskDefinitions mTasks;
  unordered_multimap<string, Table> mTables;
  const vector<ObjectType> mAvailableCavities;

  auto extractObjectTypes(const string& task) {
    vector<ObjectType> availableObjects;
    for ( const auto& item: mTasks[task] )
      if ( item.second && regex_match( item.first, regex("[A-Z0-9_]+") ) )
        availableObjects.emplace_back(item.first, item.second);
    return availableObjects;
  }

  Task generate( const TaskDefinition& def, const vector<ObjectType> availableObjects){
    Object::reset();
    vector<Object> container;
    vector<Object> objects;
    for ( auto& table: mTables )
      table.second.reset();

    

    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Objects:\n" << objects);
    // TODO

    return Task();
  }

  void sanityCheck(std::string task = "", const vector<ObjectType>* availableObjects=NULL) {
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

    if (mTasks[task]["object_count"]>=0 && availableObjects->empty() ) {
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
      : mTasks(tasks), mAvailableCavities( extractCavities( arena ) )
    {

      for (const auto& table: arena.workstations)
        mTables.emplace(table.second, Table(table.first, table.second));



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
      auto availableObjects = extractObjectTypes(task->first);
      ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Tables:\n" << mTables);
      ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Cavities:\n" << mAvailableCavities);
      ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] ObjectTypes:\n" << availableObjects);
      sanityCheck(task->first, &availableObjects);
      return generate(task->second, availableObjects);
    }
};

TaskGenerator::TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks)
  : mImpl(new TaskGeneratorImpl(arena, tasks)) {}

TaskGenerator::~TaskGenerator() { delete mImpl; }

Task TaskGenerator::operator()(string taskName) { return mImpl->operator()(taskName); }

}

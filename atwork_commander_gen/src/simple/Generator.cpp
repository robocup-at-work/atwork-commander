#include <atwork_commander_gen/GeneratorPluginInterface.h>
#include <atwork_commander_gen/DefaultConfigParser.h>

#include <atwork_commander_msgs/Task.h>
#include <atwork_commander_msgs/Object.h>
#include <atwork_commander_msgs/Workstation.h>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <sstream>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <regex>
#include <iterator>
#include <random>
#include <numeric>
#include <type_traits>

using namespace std;

namespace atwork_commander {

class Object;
class Table;

using ObjectPtr = Object*;
using TablePtr = Table*;

enum class Orientation : unsigned int {
  FREE,       ///< Orientation to be chosen by the Team
  VERTICAL,   ///< Vertical Orientation
  HORIZONTAL  ///< Horizontal Orientation
};

enum class Type : unsigned int {
  UNKNOWN,        ///< unknown object( invalid )
  OBJECT,         ///< Plain Object without special properties
  COLORED_OBJECT, ///< Colored Object
  CAVITY,         ///< PPT Cavity
  CONTAINER       ///< Container( currently RED or BLUE )
};

struct ObjectBase {
  static Type extractType(const string& typeName) {
    size_t len = typeName.length();
    char preLast = typeName[ len - 2 ];
    if( preLast == '_' ) {
      char last = typeName[ len - 1 ];
      if (  last == 'H' || last == 'V' )
        return Type::CAVITY;
      if (  last == 'G' || last == 'B' )
        return Type::COLORED_OBJECT;
    }
    if ( regex_match( typeName, regex( "CONTAINER_.*" ) ) )
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
      case ( Type::CONTAINER ): return typeName.substr( strlen("CONTAINER_") );
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

  Type type = Type::UNKNOWN;
  string form = "";
  string color = "";
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
  unsigned int id=0;
  TablePtr source = nullptr;
  TablePtr destination = nullptr;
  ObjectPtr container = nullptr;
  Object() = default;
  Object(ObjectType& type): ObjectBase(type), id(globalID++) { type--; }
  static void reset() { globalID = 0; }
};

unsigned int Object::globalID = 1;

struct Table {
  std::string name ="";
  std::string type ="";
  Table() = default;
  Table(const std::string& name, const std::string& type)
    : name(name), type(type) {}
};

template<typename T>
void errorOut(ostringstream& os, const T* v) {
  os << " ["  << *v << "]";
}

template<typename T>
void errorOut(ostringstream& os, T* v) {
  os << " [" << *v << "]";
}

template<>
void errorOut(ostringstream& os, const char* v) {
  os << " " << v;
}

template<typename T>
void errorOut(ostringstream& os, T v) {
  os << " " << v;
}

template<typename T>
void errorPass(ostringstream& os, T first) {
  errorOut(os, first);
}

template<typename T, typename... Args>
void errorPass(ostringstream& os, T first, Args... args) {
  errorOut(os, first);
  errorPass(os, args...);
}

template<typename... Args>
void errorImpl(const char* file, size_t line, const char* func, const char* msg, Args... args ) {
  ostringstream os;
  os << "Location: " << file << ":" << line << endl << "Function: " << func << endl << "Message: " << msg << endl;
  errorPass(os, args...);
  throw std::runtime_error( os.str() );
}

#define error( msg, ... )  errorImpl(__FILE__, __LINE__, __PRETTY_FUNCTION__, msg, __VA_ARGS__ );

}

ostream& operator<<(ostream& os, const atwork_commander::Type type) {
  switch ( type ) {
    case ( atwork_commander::Type::CAVITY ):         return os << "Cavity";
    case ( atwork_commander::Type::CONTAINER ):      return os << "Container";
    case ( atwork_commander::Type::COLORED_OBJECT ): return os << "Colored Object";
    case ( atwork_commander::Type::OBJECT ):         return os << "Plain Object";
    default:                                          return os << "UNKNOWN";
  }
}

ostream& operator<<(ostream& os, const atwork_commander::Orientation o) {
  switch ( o ) {
    case( atwork_commander::Orientation::VERTICAL )  : return os << "V";
    case( atwork_commander::Orientation::HORIZONTAL ): return os << "H";
    case( atwork_commander::Orientation::FREE )      : return os << "FREE";
    default                                           : return os << "UNKNOWN";
  }
}

ostream& operator<<(ostream& os, const atwork_commander::ObjectBase& type) {
  switch ( type.type ) {
    case ( atwork_commander::Type::CAVITY ):         return os << type.form << "_" << type.orientation;
    case ( atwork_commander::Type::CONTAINER ):      return os << "CONTAINER_"     << type.color;
    case ( atwork_commander::Type::COLORED_OBJECT ): return os << type.form << "_" << type.color;
    case ( atwork_commander::Type::OBJECT ):         return os << type.form;
    default:                                           return os << "UNKNOWN OBJECT TYPE";
  }
}

ostream& operator<<(ostream& os, const atwork_commander::ObjectType& type) {
  return os << ((atwork_commander::ObjectBase)type) << "(" << type.count << ")";
}


ostream& operator<<(ostream& os, const atwork_commander::Table& t) {
  return os << "Table " << t.name << "(" << t.type << "):";
}

ostream& operator<<(ostream& os, const atwork_commander::Object& o) {
  os << "Object " << atwork_commander::ObjectBase(o) << "(" << o.id << "):";
  if ( o.source )      os << " Src: " << o.source->name      << "(" << o.source->type      << ")";
  if ( o.destination ) os << " Dst: " << o.destination->name << "(" << o.destination->type << ")";
  if ( o.container )   os << " Cont: " << o.container->type   << "(" << o.container->id     << ")";
  return os;
}

ostream& operator<<(ostream& os, const vector<atwork_commander::Object>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?"\n":"");
  return os;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<T*>& v) {
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

namespace atwork_commander {
namespace task_generator {
namespace simple {

/** Task Generation Implementation
 *
 * Implements the generation of Task according to supplied configurations.
 *
 *
 *
 **/
class Generator: public GeneratorPluginInterface {

  static auto extractCavities(const ArenaDescription& arena) {
    vector<ObjectType> cavities;
      for (const auto& item: arena.cavities)
          cavities.emplace_back(item, 1);
    return cavities;
  }
  default_random_engine mRand;
  DefaultConfigParser mConfig;
  TaskDefinitions& mTasks = const_cast<TaskDefinitions&>(mConfig.tasks());
  unordered_multimap<string, Table> mTables;
  vector<ObjectType> mAvailableCavities;

  auto extractObjectTypes(const string& task) {
    vector<ObjectType> availableObjects;
    for ( const auto& item: mTasks[task].objects )
      if ( item.second && regex_match( item.first, regex("[A-Z0-9_]+") ) )
        availableObjects.emplace_back(item.first, item.second);
    return availableObjects;
  }

  void sanityCheck(std::string task, const vector<ObjectType>& availableObjects) {
    auto& params = mTasks[task].parameters;
    if (params["objects"]>=0 && availableObjects.empty() ) {
      ostringstream os;
      os << task << ": Transportation Task without allowed object defined!";
      throw runtime_error(os.str());
    }
    if (params["waypoints"]>=0 && mTables.size()<params["waypoints"] ) {
      ostringstream os;
      os << task << ": Navigation Task without enough workstations defined!";
      throw runtime_error(os.str());
    }
    if ( ( params["shelf_grasping"] || params["shelf_picking"] ) && mTables.count("SH")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving shelf requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( ( params["tt_grasping"] || params["tt_picking"] ) && mTables.count("TT")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving Rotating Table requested in Arena without it!";
      throw runtime_error(os.str());
    }
    if ( params["pp_placing"] && mTables.count("PP")==0 ) {
      ostringstream os;
      os << task << ": Transportation Task involving Precision Placement requested in Arena without it!";
      throw runtime_error(os.str());
    }
    //TODO
  }

  void sanityCheck() {
    if (mTables.size() < 2) throw runtime_error("At least two tables need to exist in the arena!");
    if (mTasks.empty()) throw runtime_error("No Tasks configured!");
    for (auto& task : mTasks ) {
      if ( task.second.parameters["objects"] == 0 && task.second.parameters["waypoints"] == 0 ) {
        ostringstream os;
        os << task.first << ": Empty Task defined!";
        throw runtime_error(os.str());
      }
      if ( task.second.parameters[ "prep_time" ] <= 0 || task.second.parameters[ "exec_time" ] <= 0 ) {
        ostringstream os;
        os << task.first << ": preparation time or execution time specified <= 0!";
        throw runtime_error(os.str());
      }
    }
  }

  vector<TablePtr> extractTablesByTypes(vector<string> types) {
    vector<TablePtr> tables;
    size_t numTables = accumulate(types.begin(), types.end(), 0, [this](size_t n, const string& s){ return n + mTables.count( s ); } );
    tables.resize( numTables );
    auto start = tables.begin();
    for ( const string& type : types ) {
      auto its = mTables.equal_range( type );
      start = transform(its.first, its.second, start, []( decltype(mTables)::value_type& t ){ return &t.second; } );
    }
    if ( tables.empty() ) {
      ostringstream os;
      os << "No tables with following types exist: [" << types << "]";
      throw runtime_error( os.str() );
    }
    return tables;
  }

  template<typename T>
  static vector<T*> toPtr(typename vector<T>::iterator start, typename vector<T>::iterator end) {
    vector<T*> ptrs( end - start );
    transform( start, end, ptrs.begin(), [](T& t){ return &t; } );
    return ptrs;
  }

  uint16_t toContainerType( const Object& o ) {
    if ( o.color == "RED" )  return atwork_commander_msgs::Object::CONTAINER_RED;
    if ( o.color == "BLUE" ) return atwork_commander_msgs::Object::CONTAINER_BLUE;
    throw runtime_error("Unknown container object enocunted!");
  }

  uint16_t toCavityType( const Object& o ) {
    if ( o.form == "F20_20" )
      switch ( o.orientation ) {
        case( Orientation::VERTICAL )  : return atwork_commander_msgs::Object::F20_20_V;
        case( Orientation::HORIZONTAL ): return atwork_commander_msgs::Object::F20_20_H;
        case( Orientation::FREE )      : return atwork_commander_msgs::Object::F20_20_F;
      }
    if ( o.form == "S40_40" )
      switch ( o.orientation ) {
        case( Orientation::VERTICAL )  : return atwork_commander_msgs::Object::S40_40_V;
        case( Orientation::HORIZONTAL ): return atwork_commander_msgs::Object::S40_40_H;
        case( Orientation::FREE )      : return atwork_commander_msgs::Object::S40_40_F;
      }
    if ( o.form == "M20" )
      switch ( o.orientation ) {
        case( Orientation::VERTICAL )  : return atwork_commander_msgs::Object::M20_V;
        case( Orientation::HORIZONTAL ): return atwork_commander_msgs::Object::M20_H;
        case( Orientation::FREE )      : return atwork_commander_msgs::Object::M20_F;
      }
    if ( o.form == "M30" )
      switch ( o.orientation ) {
        case( Orientation::VERTICAL )  : return atwork_commander_msgs::Object::M30_V;
        case( Orientation::HORIZONTAL ): return atwork_commander_msgs::Object::M30_H;
        case( Orientation::FREE )      : return atwork_commander_msgs::Object::M30_F;
      }
    if ( o.form == "M20_100" )
      switch ( o.orientation ) {
        case( Orientation::VERTICAL )  : return atwork_commander_msgs::Object::M20_100_V;
        case( Orientation::HORIZONTAL ): return atwork_commander_msgs::Object::M20_100_H;
        case( Orientation::FREE )      : return atwork_commander_msgs::Object::M20_100_F;
      }
    if ( o.form == "R20" )
      switch ( o.orientation ) {
        case( Orientation::VERTICAL )  : return atwork_commander_msgs::Object::R20_V;
        case( Orientation::HORIZONTAL ): return atwork_commander_msgs::Object::R20_H;
        case( Orientation::FREE )      : return atwork_commander_msgs::Object::R20_F;
      }
    throw runtime_error("Unknown cavity type encountered!");
  }

  uint16_t toColoredObjectType( const Object& o ) {
    if ( o.form == "F20_20" ) {
      if ( o.color == "B" ) return atwork_commander_msgs::Object::F20_20_B;
      if ( o.color == "G" ) return atwork_commander_msgs::Object::F20_20_G;
    }
    if ( o.form == "S40_40" ) {
      if ( o.color == "B" ) return atwork_commander_msgs::Object::S40_40_B;
      if ( o.color == "G" ) return atwork_commander_msgs::Object::S40_40_G;
    }
    throw runtime_error("Unknown colored object type encountered!");
  }

  uint16_t toObjectType( const Object& o ) {
    if ( o.form == "M20_100" )       return atwork_commander_msgs::Object::M20_100;
    if ( o.form == "M20" )           return atwork_commander_msgs::Object::M20;
    if ( o.form == "M30" )           return atwork_commander_msgs::Object::M30;
    if ( o.form == "R20" )           return atwork_commander_msgs::Object::R20;
    if ( o.form == "AXIS" )          return atwork_commander_msgs::Object::AXIS;
    if ( o.form == "BEARING" )       return atwork_commander_msgs::Object::BEARING;
    if ( o.form == "BEARING_BOX" )   return atwork_commander_msgs::Object::BEARING_BOX;
    if ( o.form == "DISTANCE_TUBE" ) return atwork_commander_msgs::Object::DISTANCE_TUBE;
    if ( o.form == "MOTOR" )         return atwork_commander_msgs::Object::MOTOR;
    if ( o.form == "Axis2" )         return atwork_commander_msgs::Object::Axis2;
    if ( o.form == "Bearing2" )      return atwork_commander_msgs::Object::Bearing2;
    if ( o.form == "Housing" )       return atwork_commander_msgs::Object::Housing;
    if ( o.form == "Motor2" )        return atwork_commander_msgs::Object::Motor2;
    if ( o.form == "Spacer" )        return atwork_commander_msgs::Object::Spacer;
    if ( o.form == "Wrench" )        return atwork_commander_msgs::Object::Wrench;
    if ( o.form == "Drill" )         return atwork_commander_msgs::Object::Drill;
    if ( o.form == "AllenKey" )      return atwork_commander_msgs::Object::AllenKey;
    if ( o.form == "Screwdriver" )   return atwork_commander_msgs::Object::Screwdriver;
    throw runtime_error("Unknown plain object enocunted!");
  }

  atwork_commander_msgs::Object toTaskObject( const Object& o ) {
    atwork_commander_msgs::Object object;
    switch ( o.type ) {
      case( Type::CONTAINER )     : object.object = toContainerType(o); break;
      case( Type::CAVITY )        : object.object = toCavityType(o); break;
      case( Type::COLORED_OBJECT ): object.object = toColoredObjectType(o); break;
      case( Type::OBJECT )        : object.object = toObjectType(o); break;
      default                     : throw runtime_error("Unknown object type encountered in conversion to task message");
    }
    if ( o.container ) {
      switch (o.container->type ) {
        case ( Type::CONTAINER ): object.target = toContainerType( *o.container ); break;
        case ( Type::CAVITY )   : object.target = toCavityType( *o.container ); break;
        default                 : throw runtime_error("Unknown/invalid object type to place an object in!");
      }
    }
    return object;
  }

  Task toTask(const vector<Object>& objects) {
    ROS_DEBUG_STREAM("[REFBOX] Converting generated object list to task description");
    Task task;
    task.arena_start_state.resize( mTables.size() );
    task.arena_target_state.resize( mTables.size() );

    unordered_map<const Table*, size_t> index( mTables.size() );

    size_t i=0;
    for ( const auto& table : mTables ) {
      index.emplace(&table.second, i);
      task.arena_start_state[i].name = table.second.name;
      task.arena_start_state[i].name = table.second.type;
      task.arena_target_state[i].name = table.second.name;
      task.arena_target_state[i].name = table.second.type;
      i++;
    }

    for ( const Object& o: objects ) {
      size_t srcID = index[ o.source ];
      task.arena_start_state[srcID].objects.push_back(toTaskObject(o));

      size_t dstID;
      if ( o.destination )
        dstID = index[ o.destination ];
      else
        dstID = srcID;
      task.arena_target_state[dstID].objects.push_back(toTaskObject(o));
    }

    return task;
  }

  template<typename T>
  T& uniqueSelect(typename vector<T>::iterator start, typename vector<T>::iterator& end) {
    if ( start == end )
      throw runtime_error("Tried to unique select from an empty sequence!");

    auto rand = uniform_int_distribution<size_t>( 0, end - start - 1 );
    auto selected = start + rand( mRand );
    iter_swap(selected, end - 1);
    return *--end;
  }

  static void cleanTypes(vector<ObjectType>& types) {
    auto end = remove_if(types.begin(), types.end(), [](const ObjectType& t){ return t.count == 0; });
    types.erase(end, types.end());
  }

  vector<Object*> generateCavities( TaskDefinition& def, vector<ObjectType>& availableCavities, vector<Object>::iterator start )
  {
    size_t cavitiesPerPPT = 5;
    size_t n = mTables.count( "PP" );
    size_t numCavitiesAvailable = accumulate(availableCavities.begin(), availableCavities.end(), 0ul, [](size_t n, ObjectType& t){ return t.count + n; });
    if ( n*cavitiesPerPPT > numCavitiesAvailable )
      ROS_WARN_STREAM_NAMED("generator", "[REFBOX] Not enough cavities available for " << n << " PP tables! Available: "
                            << numCavitiesAvailable << ", Needed: " << n*cavitiesPerPPT << "!");
    size_t cavitiesToGenerate = min(n*cavitiesPerPPT, numCavitiesAvailable);
    shuffle( availableCavities.begin(), availableCavities.end(), mRand );
    transform( availableCavities.begin(), availableCavities.begin() + cavitiesToGenerate, start,
               [](ObjectType& t){ return Object(t); }
             );

    uniform_int_distribution<size_t> rand(0, n-1);
    auto ppTables = extractTablesByTypes( { "PP" } );
    auto cavities = toPtr<Object>(start, start + cavitiesToGenerate);

    for (auto& cavityPtr : cavities) {
      Table* table = ppTables[ rand(mRand) ];
      cavityPtr->source = table;
    }

    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Generated Cavities:" << endl << cavities);
    return cavities;
  }
  /** \todo Implement generation of all container types on the same table**/
  vector<Object*> generateContainers( TaskDefinition& def,  vector<ObjectType>& availableObjects, vector<Object>::iterator start)
  {
    vector<string> allowedTableTypes( def.normalTableTypes );
    if ( def.parameters[ "container_on_shelf" ] ) {
      allowedTableTypes.resize( allowedTableTypes.size() + def.shTypes.size() );
      copy( def.shTypes.cbegin(), def.shTypes.cend(), allowedTableTypes.begin());
    }
    if ( def.parameters[ "container_on_tt" ] ) {
      allowedTableTypes.resize( allowedTableTypes.size() + def.ttTypes.size() );
      copy( def.ttTypes.cbegin(), def.ttTypes.cend(), allowedTableTypes.begin());
    }

    auto tables = extractTablesByTypes( allowedTableTypes );

    uniform_int_distribution<size_t> rand(0, tables.size()-1);
    auto origStart = start;
    for ( ObjectType& type : availableObjects )
      if ( type.type == Type::CONTAINER )
        for ( size_t i = 0; i < type.count; i++) {
          Object temp(type);
          Table* table = tables[ rand( mRand ) ];
          temp.source = table;
          *start++ = temp;
        }
    auto containers = toPtr<Object>( origStart, start);
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Generated Containers:" << endl << containers);
    return containers;
  }


  vector<Object*> generateObjects( TaskDefinition& def, vector<ObjectType*> availableObjects,
                                  vector<Object>::iterator start, size_t objectCount, size_t decoyCount)
  {
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Generate " << objectCount << " Objects and " << decoyCount << " Decoys");
    auto typeEnd = availableObjects.end();
    auto it = start;
    for ( size_t i = 0; i < objectCount + decoyCount; i++) {
      if ( typeEnd - availableObjects.begin() == 0)
        error("Could not generate any object", "Objects", objectCount, "Decoys", decoyCount,
              "Generated", it-start, "Types", &availableObjects);
      uniform_int_distribution<size_t> rand(0, typeEnd - availableObjects.begin() - 1);
      size_t offset = rand( mRand );
      ObjectType& type = *availableObjects[ offset ];
      *it++ = Object(type);
      if ( !type.count )
        iter_swap(availableObjects.begin() + offset, --typeEnd);
    }
    auto objects = toPtr<Object>(start, it);
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Generated Objects:" << endl << objects);
    return toPtr<Object>(start, start + objectCount);
  }

  vector<Object*> generateObjects( TaskDefinition& def, vector<ObjectType> availableObjects,
                                  vector<Object>::iterator start, size_t objectCount=1, size_t decoyCount=0)
  {
    return generateObjects(def, toPtr<ObjectType>(availableObjects.begin(), availableObjects.end()), start, objectCount, decoyCount);
  }

  template<typename F>
  vector<Object*> generateObjects( TaskDefinition& def, vector<ObjectType> availableObjects,
                                  vector<Object>::iterator start, size_t objectCount, size_t decoyCount, F&& filter) {
    vector<ObjectType*> ptrs = toPtr<ObjectType>(availableObjects.begin(), availableObjects.end());
    auto end = remove_if(ptrs.begin(), ptrs.end(), filter);
    ptrs.erase(end, ptrs.end());
    return generateObjects(def, ptrs, start, objectCount, decoyCount);

  }


  void place( TaskDefinition& def, Object& object, vector<string> tableTypes )
  {
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Place " << object << " on table of types [" << tableTypes << "]");
    vector<TablePtr> tables = extractTablesByTypes( tableTypes );
    if ( tables.empty() ) {
      ostringstream os;
      os << "Not enough tables of Types " << tableTypes << " available! Min. necessary: 1, Extracted: 0";
      throw runtime_error( os.str() );
    }
    uniform_int_distribution<size_t> rand(0, tables.size() - 1 );
    Table* table = tables[ rand( mRand ) ];
    object.destination = table;
  }

  void place( TaskDefinition& def, Object& object, Object& container )
  {
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Place " << object << " in " << container);
    if ( container.type == Type::CAVITY && def.parameters[ "ref_cavity_orientation" ] )
      container.orientation = Orientation::FREE;
    object.destination = container.source;
    object.container = &container;
  }

  void pick( TaskDefinition& def, Object& object, vector<string> tableTypes )
  {

    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX] Pick " << object << " from table of types [" << tableTypes << "]");
    vector<TablePtr> tables = extractTablesByTypes( tableTypes );
    if ( tables.empty() ) {
      ostringstream os;
      os << "Not enough tables of Types " << tableTypes << " available! Min. necessary: 1, Extracted: 0";
      throw runtime_error( os.str() );
    }
    uniform_int_distribution<size_t> rand(0, tables.size() - 1 );
    Table* table = tables[ rand( mRand ) ];
    object.source = table;
  }

  Task generateImpl( const string& taskName){

          auto&  def            = mTasks[ taskName ];
          auto&  params         = def.parameters;
    const auto&  normalTables   = def.normalTableTypes;
    const auto&  turnTables     = def.ttTypes;
    const auto&  shelfs         = def.shTypes;
    const auto&  precisionTable = def.ppTypes;
    Object::reset();
    vector<Object> objects;

    auto availableObjects = extractObjectTypes( taskName );
    vector<ObjectType> availableCavities = mAvailableCavities; // TODO: filter according to allowed Objects from Task
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Tables:\n" << mTables);
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Cavities:\n" << mAvailableCavities);
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] ObjectTypes:\n" << availableObjects);

    sanityCheck(taskName, availableObjects);
    auto seedIt = params.find("seed");
    if ( seedIt != params.end() ) mRand.seed(seedIt->second);


    // PP fill
    // generate places on PP
    // Container generate and distribute
    // generate places on Container
    // generate places on shelf
    // generate remaining places
    // generate picks on shelf
    // generate picks on rt
    // generate remaining picks


    size_t objectCount = accumulate(availableCavities.begin(), availableCavities.end(), 0,
                                    [](size_t n, const ObjectType& t){ return n + t.count; } );
    objectCount += accumulate(availableObjects.begin(), availableObjects.end(), 0,
                                    [](size_t n, const ObjectType& t){ return n + t.count; } );

    objects.resize(objectCount);
    auto start = objects.begin();

    auto cavities = generateCavities( def, availableCavities, start);
    start += cavities.size();
    cleanTypes( availableCavities );
    if ( cavities.size() < params[ "pp_placing" ] ) {
      ostringstream os;
      os << "Not enough cavities generated! Generated " << cavities.size() << ", min Necessary: " << params[ "pp_placing" ] << "!";
      throw runtime_error(os.str());
    }
    auto end = cavities.end();
    for ( size_t i = 0; i < params[ "pp_placing" ]; i++ ) {
      Object* selected = uniqueSelect<Object*>(cavities.begin(), end);
      auto filter = [selected](const ObjectType* t){ return t->form != selected->form; };
      auto genObjects = generateObjects(def, availableObjects, start++, 1, 0, filter);
      if ( genObjects.empty() ) {
        throw runtime_error("Did not generate any objects!");
      }
      place( def, *genObjects.front(), *selected );
    }
    cleanTypes( availableObjects );

    auto containers = generateContainers( def, availableObjects, start );
    start += containers.size();
    cleanTypes( availableObjects );
    if ( containers.size() < 1 && params[ "container_placing" ] ) {
      ostringstream os;
      os << "Not enough containers generated! Generated " << containers.size() << ", min Necessary: 1!";
      throw runtime_error(os.str());
    }
    auto rand = uniform_int_distribution<size_t>( 0, containers.size()-1 );
    auto genObjects = generateObjects(def, availableObjects, start, params[ "container_placing" ] );
    start += params[ "container_placing" ];
    for ( Object* objPtr: genObjects)
      place( def, *objPtr, *containers[ rand(mRand) ] );

    size_t created = accumulate(objects.begin(), objects.end(), 0,
                                [](size_t n, const Object& o){ return n + (o.type==Type::OBJECT || o.type == Type::COLORED_OBJECT ? 1 : 0); } );
    genObjects = generateObjects(def, availableObjects, start, params[ "objects" ] - created, params[ "decoys"] );
    cleanTypes( availableObjects );
    start += params[ "objects" ] - created + params[ "decoys" ];

    size_t shelfes_placing = params[ "shelfes_placing" ];
    size_t rt_placing = params[ "tt_placing" ];
    size_t shelfes_picking = params[ "shelfes_picking" ];
    size_t rt_picking = params[ "tt_picking" ];

    for ( Object* objPtr: genObjects ) {
      if ( shelfes_placing ) {
        place( def, *objPtr, shelfs );
        shelfes_placing--;
        continue;
      }
      if ( rt_placing ) {
        place( def, *objPtr, turnTables );
        rt_placing--;
        continue;
      }
      place(def, *objPtr, normalTables );
    }

    for ( Object* objPtr: genObjects ) {
      if ( shelfes_picking ) {
        pick( def, *objPtr, shelfs );
        shelfes_picking--;
        continue;
      }
      if ( rt_picking ) {
        pick( def, *objPtr, turnTables );
        rt_picking--;
        continue;
      }
      pick(def, *objPtr, normalTables );
    }

    for (auto it = objects.begin(); it < start; it++)
      if ( !it->source )
        pick(def, *it, normalTables );

    objects.erase(start, objects.end());

    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Objects:\n" << objects);
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Available Objects:\n" << availableObjects);
    ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Available Cavities:\n" << availableCavities);

    return toTask(objects);
  }

  public:
    virtual void onInit(const std::string& arenaConfig, const std::string& taskConfig)
    {
      try {
        mConfig.reload(arenaConfig, taskConfig);
        mAvailableCavities = extractCavities( mConfig.arena() );
        for (const auto& table: mConfig.arena().workstations)
          mTables.emplace(table.second, Table(table.first, table.second));
        sanityCheck();
      }
      catch(const exception& e) {
        ROS_FATAL_STREAM_NAMED("generator", "[REFBOX-GEN] Error during initialization of plugin: " << e.what());
        throw e;
      }
    }

    /** \todo Implement **/
    virtual bool check( const Task& task ) const {
      try {
        //TODO: do checks
        return !task.arena_start_state.empty() && !task.arena_target_state.empty()  ;
      } catch(const std::exception& e) {
        ROS_DEBUG_STREAM_NAMED("generator", "[REFBOX-GEN] Exception occured during check: " << e.what());
        return false;
      } catch(...) {
        return false;
      }
    }

    virtual Task generate(const std::string& taskName) {
      auto taskIt = find_if( mTasks.begin(), mTasks.end(),
                                 [ taskName ]( const auto& item ){ return item.first == taskName; }
                               );
      if ( taskIt == mTasks.end() ) {
        ostringstream os;
        os << "No Task " << taskName << " configured. Valid tasks are: ";
        for ( const auto& item: mTasks )
          os << item.first << " ";
        throw runtime_error( os.str() );
      }
      Task task = generateImpl( taskIt->first );
      task.prep_time = ros::Duration ( taskIt->second.parameters[ "prep_time" ]*60 );
      task.exec_time = ros::Duration ( taskIt->second.parameters[ "exec_time" ]*60 );
      check( task );
      return task;
    }

    virtual ConfigParserInterface& config() { return mConfig; }

};

}
}
}

PLUGINLIB_EXPORT_CLASS(atwork_commander::task_generator::simple::Generator, atwork_commander::task_generator::GeneratorPluginInterface);

#include <atwork_commander_gen/TaskGenerator.h>

#include <atwork_commander_msgs/Task.h>
#include <atwork_commander_msgs/Object.h>
#include <atwork_commander_msgs/Workstation.h>

#include <ros/console.h>

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

template<typename T, size_t n>
ostream& operator<<(ostream& os, const vector<array<T,n>>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i];
  return os << endl;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<vector<T>>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i];
  return os << endl;
}

template<typename T, size_t n>
ostream& operator<<(ostream& os, const array<T,n>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i] << (i==vec.size()-1?"":" ");
  return os << endl;
}

namespace atwork_commander {

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
          cavities.emplace_back(item, 1);
    return cavities;
  }

  default_random_engine mRand;
  TaskDefinitions mTasks;
  unordered_multimap<string, Table> mTables;
  const vector<ObjectType> mAvailableCavities;

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
      task.arena_start_state[i].workstation_name = table.second.name;
      task.arena_target_state[i].workstation_name = table.second.name;
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

//-------------------------------------------------------------------------------------------------------------------------
//Beware Jurek code below this line >,<

        std::vector<std::string> mTableMapping;
        std::vector<unsigned int> mTables;
        std::vector<unsigned int> mTables0;
        std::vector<unsigned int> mTables5;
        std::vector<unsigned int> mTables10;
        std::vector<unsigned int> mTables15;
        std::vector<unsigned int> mConveyors;
        std::vector<unsigned int> mPpts;
        std::vector<unsigned int> mShelfs;
        std::vector<unsigned int> mWaypoints;
        std::vector<unsigned int> mObjects;
        std::vector<unsigned int> mPptObjects;
        std::vector<size_t> mAllTables;

        int mdiscipline;
        static const size_t obj_id = 0;
        static const size_t src_id = 1;
        static const size_t dst_id = 2;
        static const size_t cont_id = 3;
        static const size_t obj_id_id = 4;

        static const size_t blue = 0;
        static const size_t red = 1;

        static const size_t tables0_id = 0;
        static const size_t tables5_id = 1;
        static const size_t tables10_id = 2;
        static const size_t tables15_id = 3;
        static const size_t conveyors_id = 4;
        static const size_t ppts_id = 5;
        static const size_t shelfs_id = 6;
        std::map<std::string, int> paramBNT;
        std::map<std::string, int> paramBTT3;
        std::map<std::string, int> paramFinal;
        int estimatet_active;
        bool paramContainerInShelf;
        bool paramContainerOnPpt;
        bool paramContainerOnTurntable;
        bool paramFlexibleHeight;
        std::vector<std::array<size_t, 3>> container_ids;
        std::vector<std::vector<size_t>> validpicks;
        std::vector<size_t> picksleft;
        std::unordered_map<size_t, size_t> mTableTypes;

        size_t tabletypes;
using run = vector<array<int, 5>>;

  void debugAll(const string info, const run &tasks) {
    cout<<info<<"\n===========================\n";
    cout<<"mAllTables\n"<<mAllTables;
    cout<<"validpicks\n"<<validpicks;
    cout<<"picksleft\n"<<picksleft;
    cout<<"tabletypes "<<tabletypes<<"\n";
    cout<<"container_ids\n"<<container_ids;
    cout<<"tasks\n"<<tasks;
  }

  void debug_tasks(const string info, const run &tasks) {
    ostringstream os;
    for(size_t i=0; i<tasks.size(); ++i) {
      os << tasks.at(i).at(obj_id)<<" "<<tasks.at(i).at(src_id)<<" "<<tasks.at(i).at(dst_id)<<" "<<tasks.at(i).at(cont_id)<<" "<<tasks.at(i).at(obj_id_id) << endl;
    ROS_DEBUG(os.str());
    }
  }

  /* obj, src, dst, cont
   * BNT: location, orientation [0,1,2,3], time [ca. 1-5s]
   * objects is the number of tasks
   */

  // for every task set random object type
  // if there are already entries select ppt objects only
  void generate_objects(run &tasks) {
    const size_t objects = mObjects.size();
    const size_t pptobjects = mPptObjects.size();
    const size_t count = tasks.size();
    srand(time(NULL));
    for(size_t i=0; i<count; ++i) {
      if(tasks.at(i).at(obj_id) == -1) {
        if(tasks.at(i).at(dst_id) == -1) {
          size_t obj = rand() % objects;
          tasks.at(i).at(obj_id) = mObjects.at(obj);
        }
        else {
          size_t pptobj = rand() % pptobjects;
          tasks.at(i).at(obj_id) = mPptObjects.at(pptobj);
        }
      }
    }
  }

  // retuns an ascending order of int 0,..., n-1
  std::vector<size_t> order (size_t n) {
    std::vector<size_t> up(n);
    for(size_t i=0; i<n; ++i) {
      up.at(i) = i;
    }
    return up;
  }

  /* Input: integers n and k
   * Output: vector with k random entries between 0<= entry <n in ascending order
   */
  void variation(vector<size_t> position, size_t k, vector<size_t> &vec_k, vector<size_t> &vec_n_k) {
    size_t n = position.size();
    vec_k.resize(k);
    vec_n_k.resize(n-k);
    size_t a;
    srand(time(NULL));
    for(size_t i=0; i<k; ++i) {
      a = rand() % (n-i);
      std::swap(position.at(i), position.at(n-a-1));
    }
    copy(position.begin(), position.begin()+k, vec_k.begin());
    copy(position.begin()+k, position.end(), vec_n_k.begin());
  }

  // if the second vector isn't needed, this function can be used as an adapter
  void variation(vector<size_t> &positions, size_t k, vector<size_t> &vec_k) {
    vector<size_t> trash;
    variation(positions, k, vec_k, trash);
  }

  size_t get_container_id(size_t table, size_t color) {
    // if there already is a container of this color on the same table
    for(size_t i=0; i<container_ids.size(); ++i) {
      if (container_ids.at(i).at(0) == table && container_ids.at(i).at(1) == color) {
        return container_ids.at(i).at(2);
      }
    }
    //else get new id from the worldmodel
    if(color == blue) {
      size_t new_id = insertContainer(-1, robotto_msgs::BLUE_CONTAINER, table);
      std::array<size_t, 3> id = {table, blue, new_id};
      container_ids.push_back(id);
      return id.at(2);
    }
    else if(color == red) {
      size_t new_id = insertContainer(-1, robotto_msgs::RED_CONTAINER, table);
      std::array<size_t, 3> id = {table, red, new_id};
      container_ids.push_back(id);
      return id.at(2);
    }
    else {throw 229;}
  }

  void initialize_mAllTables() {
    size_t size = mTables0.size() + mTables5.size() + mTables10.size()
            + mTables15.size() + mConveyors.size()
            + mPpts.size() + mShelfs.size();
    mAllTables.resize(size);
    auto end = copy(mTables0.begin(), mTables0.end(), mAllTables.begin());
    end = copy(mTables5.begin(), mTables5.end(), end);
    end = copy(mTables10.begin(), mTables10.end(), end);
    end = copy(mPpts.begin(), mPpts.end(), end);
    end = copy(mConveyors.begin(), mConveyors.end(), end);
    end = copy(mTables15.begin(), mTables15.end(), end);
    end = copy(mShelfs.begin(), mShelfs.end(), end);
    /*
    std::cout<<"Tables0: "<<mTables0;
    std::cout<<"Tables5: "<<mTables5;
    std::cout<<"Tables10: "<<mTables10;
    std::cout<<"Ppts: "<<mPpts;
    std::cout<<"Conveyors: "<<mConveyors;
    std::cout<<"Tables15: "<<mTables15;
    std::cout<<"Shelfs: "<<mShelfs;
    */
  }

  // fills in all for every task every table where it isn't placed
  void initialize_validpicks(run &tasks) {
    size_t objects = paramFinal["objects"];
    size_t tables = mAllTables.size();
    validpicks.resize(objects);
    for(size_t i=0; i<objects; ++i) {
      validpicks.at(i).resize(0);
      for(size_t j=0; j<tables; ++j) {
        size_t table = mAllTables.at(j);
        if((table != size_t(tasks.at(i).at(dst_id))) && (picksleft.at(mTableTypes.at(table)) > 0)) {	// all picking tables exept the table for placing are valid
          validpicks.at(i).push_back(table);
        }
      }
    }
  }

  // reads how many picks from every table have to be done
  void initialize_picksleft() {
    picksleft.resize(tabletypes);
    picksleft.at(tables0_id) = paramFinal["pick_tables0"];
    picksleft.at(tables5_id) = paramFinal["pick_tables5"];
    picksleft.at(tables10_id) = paramFinal["pick_tables10"];
    picksleft.at(tables15_id) = paramFinal["pick_tables15"];
    picksleft.at(conveyors_id) = paramFinal["pick_turntables"];
    picksleft.at(ppts_id) = paramFinal["pick_cavity_plattforms"];
    picksleft.at(shelfs_id) = paramFinal["pick_shelf"];
  }


  void initialize_mTableTypes() {
    for(size_t table : mTables0) {
      mTableTypes[table] = tables0_id;
    }
    for(size_t table : mTables5) {
      mTableTypes[table] = tables5_id;
    }
    for(size_t table : mTables10) {
      mTableTypes[table] = tables10_id;
    }
    for(size_t table : mTables15) {
      mTableTypes[table] = tables15_id;
    }
    for(size_t table : mConveyors) {
      mTableTypes[table] = conveyors_id;
    }
    for(size_t table : mPpts) {
      mTableTypes[table] = ppts_id;
    }
    for(size_t table : mShelfs) {
      mTableTypes[table] = shelfs_id;
    }
  }

  // coputes the sum of all entries in a vector
  size_t sum_vector(vector<size_t> & vec) {
    size_t sum = 0;
    for(size_t i=0; i<vec.size(); ++i) {
      sum += vec.at(i);
    }
    return sum;
  }

  // finds the index with the shortest list of validpicks
  size_t shortest_list(size_t &min) {						// the lenght of the shortest list as refference
    size_t minindex;													// define minindex
    for(size_t i=0; i<validpicks.size(); ++i) {							// for all other table types
      if(validpicks.at(i).size() != 0) {								// if there are valid picks left
        if(validpicks.at(i).size() < min) {							// if the list is shorter than the actual minimum
          min = validpicks.at(i).size();							// update min
          minindex = i;											// and minindex
        }
      }
    }
    return minindex;													// returns th index of the shortest list
  }

  void update_validpicks(size_t type_id) {
    if(picksleft.at(type_id) == 0) {									// if no picks from this type left => picking from
      for(size_t i=0; i<validpicks.size(); ++i) {						// this table type is no more valid
        auto end = remove_if(validpicks.at(i).begin(), validpicks.at(i).end(),  [type_id, this](size_t table_id){
          return type_id == mTableTypes.at(table_id);
        });
        validpicks.at(i).erase(end, validpicks.at(i).end());
      }
    }
  }

  Task generate( const string& taskName){
    // initialize tasks
    run tasks(paramFinal["objects"], {-1, -1, -1, -1, -1});
    size_t shelfs = mShelfs.size();
    size_t tables = mTables.size();
    size_t ppts = mPpts.size();
    size_t conveyors = mConveyors.size();
    size_t containers = paramFinal["B_Container"] + paramFinal["R_Container"];
    size_t table0 = mTables0.size();
    size_t table5 = mTables5.size();
    size_t table10 = mTables10.size();
    size_t table15 = mTables15.size();

    if(paramFinal["FlexibleHeight"] == true) {
      tabletypes = 8;
    } else {
      tabletypes = 7;
    }

    // check validity
    if (mObjects.size() == 0) {throw 220;}																			// No Objects
    if (shelfs == 0 && (paramFinal["pick_shelf"] > 0 || paramFinal["place_shelf"] > 0)) {throw 221;}				// No shelfpick/place without shelf
    if (tables == 0 && paramFinal["objects"] > paramFinal["pick_shelf"]) {throw 222;}								// No tables
    if (shelfs + tables + conveyors + paramFinal["B_Container"] + paramFinal["R_Container"] <= 1) {throw 223;}		// pick = place
    if (paramContainerInShelf == true) {
      if (tables + shelfs == 0 && paramFinal["B_Container"] + paramFinal["R_Container"] > 0) {throw 224;}			// No tables, no shelfs, no conveyors, No container place in air
    }
    else if(tables == 0 && paramFinal["B_Container"] + paramFinal["R_Container"] > 0) {throw 225;}					// No tables No container place in air
    if (ppts == 0 && paramFinal["place_cavity_plattforms"] > 0) {throw 226;}										// No cavity plattforms
    if (conveyors == 0 && (paramFinal["pick_turntables"] > 0 || paramFinal["place_turntbales"] > 0)) {throw 227;}	// No conveyors
    if (table0 == 0 && paramFinal["pick_tables0"] > 0) {throw 231;}													// No tables0
    if (table5 == 0 && paramFinal["pick_tables5"] > 0) {throw 232;}													// No tables5
    if (table10 == 0 && paramFinal["pick_tables10"] > 0) {throw 233;}												// No tables10
    if (table15 == 0 && paramFinal["pick_tables15"] > 0) {throw 234;}												// No tables15
    if (paramFinal["pick_cavity_plattforms"] > 0) {throw 235;}														// Picks from cavity plattform are not implemented yet

    /* select #place_shelf + #place_turntables + #ppts random tasks
     * select #place_shelf + #place_turntables of these
     * the rest of the places are on a random turntable
     * select #place_shelf of these and set place to a random shelf
     * the rest of the places are into a random ppt
     *
     * the vectors contains the indexes of tasks for the places
     */
    std::vector<size_t>position = order(paramFinal["objects"]);
    std::vector<size_t>normalplace;
    std::vector<size_t>specialplace;
    std::vector<size_t>placeShelfTurntable;
    std::vector<size_t>placeShelf;
    std::vector<size_t>placeTurntable;
    std::vector<size_t>placePpt;
    std::vector<size_t>container;
    std::vector<size_t>b_container;
    std::vector<size_t>r_container;
    size_t specialplaces = paramFinal["place_shelf"] + paramFinal["place_turntables"] + paramFinal["place_cavity_plattforms"];
    size_t shelfTurntables = paramFinal["place_shelf"] + paramFinal["place_turntables"];
    size_t a;
    variation(position, specialplaces, specialplace, normalplace);
    variation(specialplace, shelfTurntables ,placeShelfTurntable, placePpt);
    variation(placeShelfTurntable, paramFinal["place_shelf"], placeShelf, placeTurntable);
    /*
    std::cout<<"placeShelf "<<placeShelf;
    std::cout<<"placeTurntable "<<placeTurntable;
    std::cout<<"placePpt "<<placePpt;
    std::cout<<"normalplace "<<normalplace;
    */

    // write the Ppts as destinations to the tasks
    for(size_t i=0; i<placePpt.size(); ++i) {
      a = rand() % ppts;
      tasks.at(placePpt.at(i)).at(dst_id) = mPpts.at(a);
    }

    // generate the objects, pptobjects seperately from the others
    // in the tasks wich already have vaild entries select ppt objects only
    generate_objects(tasks);

    // write all the other distinations to the tasks
    for(size_t i=0; i<placeShelf.size(); ++i) {
      a = rand() % shelfs;
      tasks.at(placeShelf.at(i)).at(dst_id) = mShelfs.at(a);
    }
    for(size_t i=0; i<placeTurntable.size(); ++i) {
      a = rand() % conveyors;
      tasks.at(placeTurntable.at(i)).at(dst_id) = mConveyors.at(a);
    }
    // PLACES NOCH BEARBEITEN KEINE PLACES, FALLS KEINE PICK VON DER TISCHHÖHE
    for(size_t i=0; i<normalplace.size(); ++i) {
      a = rand() % tables;
      tasks.at(normalplace.at(i)).at(dst_id) = mTables.at(a);
    }

    // collect all valid places to place a Container
    // If the respective setting is active add these dst to the valid containerplaces
    vector<size_t>valid_containerplaces = normalplace;
    vector<size_t>mBContainer, mRContainer;
    if(paramContainerInShelf == true) {
      copy(placeShelf.begin(), placeShelf.end(), back_inserter(valid_containerplaces));
    }
    if(paramContainerOnTurntable == true) {
      copy(placeTurntable.begin(), placeTurntable.end(), back_inserter(valid_containerplaces));
    }
    if(paramContainerOnPpt == true) {
      copy(placePpt.begin(), placePpt.end(), back_inserter(valid_containerplaces));
    }

    variation(valid_containerplaces, containers, container);
    variation(container, paramFinal["B_Container"], b_container, r_container);

    for(size_t i=0; i<size_t(paramFinal["B_Container"]); ++i) {
      size_t table = tasks.at(b_container.at(i)).at(dst_id);
      size_t blue_container_id = get_container_id(table, blue);
      tasks.at(b_container.at(i)).at(cont_id) = blue_container_id;
    }
    for(size_t i=0; i<size_t(paramFinal["R_Container"]); ++i) {
      size_t table = tasks.at(r_container.at(i)).at(dst_id);
      size_t red_container_id = get_container_id(table, red);
      tasks.at(r_container.at(i)).at(cont_id) = red_container_id;
    }

    initialize_mAllTables();
    initialize_picksleft();
    initialize_mTableTypes();
    initialize_validpicks(tasks);										// generate a list of valid picks for each task
    //debugAll("after initialize_mTableTypes", tasks);

    while(sum_vector(picksleft) > 0) {
      size_t min = numeric_limits<size_t>::max();						// set min to the mximum value of size_t;
      size_t index = shortest_list(min);								// min is a refference
      if (min == numeric_limits<size_t>::max()) {throw 230;}			// if all validpick vectors are empty
      a = rand() % min;												// select a random valid pick for these table
      size_t table = validpicks.at(index).at(a);						// set table to the ID for these table
      tasks.at(index).at(src_id) = table;
      validpicks.at(index).resize(0);									// task has table => no validpicks left
      size_t type_id = mTableTypes.at(table);							// set type to the type of table (mTableTypes is a map)
      --picksleft.at(type_id);										// reduce the number of picks left
      update_validpicks(type_id);										// update the lists of vaild picks
      //debugAll("taskgenerierung", tasks);
    }

    for(int i=0; i<paramFinal["decoys"]; ++i) {       // add tasks for the decoys
      size_t local = rand() % tables;
      tasks.push_back({-1,int(mAllTables.at(local)),-1,-1,-1});
    }
    generate_objects(tasks);                            // generate objects for the decoys
    //debugAll("after initialize_mTableTypes", tasks);
    return toTask(tasks);
  }

  public:
    TaskGeneratorImpl(const ArenaDescription& arena, const TaskDefinitions& tasks)
      : mTasks(tasks), mAvailableCavities( extractCavities( arena ) )
    {

      for (const auto& table: arena.workstations)
        mTables.emplace(table.second, Table(table.first, table.second));



      sanityCheck();
    }


    void checkPickCounts(const run& tasks) {
      std::vector<size_t> counter(tabletypes,0);
      for(size_t i=0; i<tasks.size(); ++i) {
        if(tasks.at(i).at(src_id) != -1) {
          counter.at(mTableTypes.at(tasks.at(i).at(src_id)))++;
        }
      }
      if(counter.at(tables0_id) != size_t(paramFinal["pick_tables0"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_tables0"]));
        errormessage += " wanted, but " + to_string(counter.at(tables0_id)) + "found.\n";
        throw errormessage;
      }
      if(counter.at(tables5_id) != size_t(paramFinal["pick_tables5"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_tables5"]));
        errormessage += " wanted, but " + to_string(counter.at(tables5_id)) + "found.\n";
        throw errormessage;
      }
      if(counter.at(tables10_id) != size_t(paramFinal["pick_tables10"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_tables10"]));
        errormessage += " wanted, but " + to_string(counter.at(tables10_id)) + "found.\n";
        throw errormessage;
      }
      if(counter.at(tables15_id) != size_t(paramFinal["pick_tables15"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_tables15"]));
        errormessage += " wanted, but " + to_string(counter.at(tables15_id)) + "found.\n";
        throw errormessage;
      }
      if(counter.at(conveyors_id) != size_t(paramFinal["pick_turntables"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_turntables"]));
        errormessage += " wanted, but " + to_string(counter.at(conveyors_id)) + "found.\n";
        throw errormessage;
      }
      if(counter.at(shelfs_id) != size_t(paramFinal["pick_shelfs"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_shelfs"]));
        errormessage += " wanted, but " + to_string(counter.at(shelfs_id)) + "found.\n";
        throw errormessage;
      }
      if(counter.at(ppts_id) != size_t(paramFinal["pick_ppts"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(paramFinal["pick_ppts"]));
        errormessage += " wanted, but " + to_string(counter.at(ppts_id)) + "found.\n";
        throw errormessage;
      }
      std::cout<<"All table types occure with correct multiplicities.\n";
    }

    bool check( const Task& task ) const {
      const run taskToRun = fromTask(task);
      debug_tasks("final tasks", taskToRun);
      checkPickNeqPlace(taskToRun);
      checkPickCounts(taskToRun);
    }

    Task operator()(std::string taskName) {
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
      Task task = generate( taskIt->first );
      task.prep_time = ros::Duration ( taskIt->second.parameters[ "prep_time" ]*60 );
      task.exec_time = ros::Duration ( taskIt->second.parameters[ "exec_time" ]*60 );
      check( task );
      return task;
    }

};

TaskGenerator::TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks)
  : mImpl(new TaskGeneratorImpl(arena, tasks)) {}

TaskGenerator::~TaskGenerator() { delete mImpl; }

Task TaskGenerator::operator()(string taskName)   { return mImpl->operator()(taskName); }

/** \todo Implement! **/
bool TaskGenerator::check(const Task& task) const { return mImpl?mImpl->check(task):false; }

}

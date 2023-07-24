#include "../Generator.h"
#include "TaskConverter.h"

#include <atwork_commander_gen/DefaultConfigParser.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <iterator>
#include <numeric>
#include <random>
#include <sstream>

template<typename T>
std::ostream& operator<<(std::ostream& o, const std::set<T*>& v) {
  o << "[ ";
  for(const T* t: v)
    o << *t << " ";
  return o << "]";
}

namespace atwork_commander {
namespace task_generator {
namespace jurek {

/** Task Generation Implementation
 *
 * Implements the generation of Task according to supplied configurations.
 **/
class Generator : public GeneratorPluginInterface {

  static auto extractCavities(const ArenaDescription& arena) {
    vector<ObjectType> cavities;
      for (const auto& item: arena.cavities)
          cavities.emplace_back(item, 1);
    return cavities;
  }

  random_device mRD;
  mt19937_64 mRand=mt19937_64(mRand);

  DefaultConfigParser mConfig;
  TaskDefinitions& mTasks = const_cast<TaskDefinitions&>(mConfig.tasks());
  unordered_multimap<string, Table> mTables;
  vector<ObjectType> mAvailableCavities;
  vector<ObjectType> mAvailableObjects;
  vector<std::array<size_t, 3>> container_ids;
  vector<string> mTablesInverse;

  auto extractObjectTypes(const string& task) {
    vector<ObjectType> availableObjects;
    for ( const auto& item: mTasks[task].objects )
      if ( item.second && regex_match( item.first, regex("[A-Za-z0-9_]+") ) )
        availableObjects.emplace_back(item.first, item.second);
    return availableObjects;
  }

  auto extractObjectTypes(const ArenaDescription& arena) {
    vector<ObjectType> availableObjects;
    for ( const auto& item: arena.objects )
      if ( item.second && regex_match( item.first, regex("[A-Za-z0-9_]+") ) )
        availableObjects.emplace_back(item.first, item.second);
    return availableObjects;
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

  uint16_t toContainerType( const ObjectType& o ) {
    if ( o.color == "RED" )  return atwork_commander_msgs::Object::CONTAINER_RED;
    if ( o.color == "BLUE" ) return atwork_commander_msgs::Object::CONTAINER_BLUE;
    throw runtime_error("Unknown container object enocunted!");
  }

  static uint16_t toCavityType( const ObjectType& o ) {
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

  uint16_t toColoredObjectType( const ObjectType& o ) {
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

  uint16_t toObjectType( const ObjectType& o ) {
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

  atwork_commander_msgs::Object toTaskObject( const ObjectType& o ) {
    atwork_commander_msgs::Object object;
    switch ( o.type ) {
      case( Type::CONTAINER )     : object.object = toContainerType(o); break;
      case( Type::CAVITY )        : object.object = toCavityType(o); break;
      case( Type::COLORED_OBJECT ): object.object = toColoredObjectType(o); break;
      case( Type::OBJECT )        : object.object = toObjectType(o); break;
      default                     : throw runtime_error("Unknown object type encountered in conversion to task message");
    }
    return object;
  }
  atwork_commander_msgs::Object toTaskObject( const Object& o ) {
    atwork_commander_msgs::Object object = toTaskObject((const ObjectType&)o);
    if ( o.container ) {
      switch (o.container->type ) {
        case ( Type::CONTAINER ): object.target = toContainerType( *o.container ); break;
        case ( Type::CAVITY )   : object.target = toCavityType( *o.container ); break;
        default                 : throw runtime_error("Unknown/invalid object type to place an object in!");
      }
    }
    return object;
  }

  size_t toCavity(size_t id, Orientation o) const {
    size_t cavity;
    switch(id) {
      case(atwork_commander_msgs::Object::F20_20_B):
      case(atwork_commander_msgs::Object::F20_20_G): cavity = atwork_commander_msgs::Object::F20_20_H; break;
      case(atwork_commander_msgs::Object::S40_40_B):
      case(atwork_commander_msgs::Object::S40_40_G): cavity = atwork_commander_msgs::Object::S40_40_H; break;
      case(atwork_commander_msgs::Object::M20): cavity = atwork_commander_msgs::Object::M20_H; break;
      case(atwork_commander_msgs::Object::M30): cavity = atwork_commander_msgs::Object::M30_H; break;
      case(atwork_commander_msgs::Object::M20_100): cavity = atwork_commander_msgs::Object::M20_100_H; break;
      case(atwork_commander_msgs::Object::R20): cavity = atwork_commander_msgs::Object::R20_H; break;
      default: throw runtime_error("Object without existing cavity to be placed on PPT ("+to_string(id)+")");
    }
    return cavity+(size_t)o;
  }

  Task toTask(const vector<array<int, 5>>& run) const {
    Task task;
    ROS_DEBUG_STREAM("Starting task conversion with inverse tables "<<mTablesInverse);
    task.arena_start_state.resize(mTablesInverse.size()-1);
    task.arena_target_state.resize(mTablesInverse.size()-1);
    for( unsigned int i=1; i < mTablesInverse.size(); i++ ) {
      task.arena_start_state[i-1].name = mTablesInverse[i];
      task.arena_start_state[i-1].type = mConfig.arena().workstations.at(mTablesInverse[i]);
      task.arena_target_state[i-1].name = mTablesInverse[i];
      task.arena_target_state[i-1].type = mConfig.arena().workstations.at(mTablesInverse[i]);
    }
    for( const array<size_t, 3>& cont : container_ids) {
      atwork_commander_msgs::Object o;
      o.object = cont[1];
      o.decoy = false;
      o.target = atwork_commander_msgs::Object::EMPTY;
      task.arena_start_state[cont[0]-1].objects.push_back(o);
      task.arena_target_state[cont[0]-1].objects.push_back(o);
    }
    for(const array<int, 5>& obj : run) {
      ROS_DEBUG_STREAM("Converting object: "<<obj);
      atwork_commander_msgs::Object o;
      o.object = obj[obj_id];
      o.decoy = false;
      if( obj[cont_id] == -1)
        o.target = atwork_commander_msgs::Object::EMPTY;
      else
        o.target = container_ids[obj[cont_id]-1][1];
      if(o.target == atwork_commander_msgs::Object::EMPTY &&
         any_of(mTables.equal_range("PP").first, mTables.equal_range("PP").second, [this, obj](const pair<string, Table>& t){return t.second.name == mTablesInverse[obj[dst_id]];})) {
        o.target = toCavity(o.object, Orientation::FREE);
      }
      if ( obj[dst_id] == -1 ) {
        o.decoy = true;
        task.arena_target_state[obj[src_id]-1].objects.push_back(o);
      } else
        task.arena_target_state[obj[dst_id]-1].objects.push_back(o);
      task.arena_start_state[obj[src_id]-1].objects.push_back(o);
    }
    ROS_DEBUG_STREAM("Task after conversion:"<<std::endl<<task);
    return task;
  }

  unsigned int contId = 1;


//-------------------------------------------------------------------------------------------------------------------------
//Beware Jureks code below this line >,<

  string printError(int error) const {
    switch (error) {
            case 100 : return "Undefined discipline";
            case 200 : return "[BTT3] No objects: Can't generate tasks without objects";
            case 201 : return "[BTT3] No shelfs: Can't generate shelf tasks without shelf";
            case 202 : return "[BTT3] No tables: Can't generate more tasks than pick_shelfs without tables";
            case 203 : return "[BTT3] infeasible constraints for task creation: place = pick";
            case 204 : return "[BTT3] No tables, no shelfs: Can't place Container in air";
            case 205 : return "[BTT3] No tables: Can't place Container in air";
            case 210 : return "[BNT] No waypoints";
            case 211 : return "[BNT] No tables";
            case 220 : return "[Final] No objects";
            case 221 : return "[Final] No shelfs: Can't generate shelf tasks without shelf";
            case 222 : return "[Final] No tables: Can't generate more tasks than pick_shelfs without tables";
            case 223 : return "[Final] infeasible constraints for task creation: place = pick";
            case 224 : return "[Final] No tables, no shelfs: Can't place Container in air";
            case 225 : return "[Final] No tables: Can't place Container in air";
            case 226 : return "[Final] No cavity plattforms: Can't generate cavity plattform tasks without cavity plattforms";
            case 227 : return "[Final] No conveyers: Can't generate conveyer tasks without conveyers";
            case 228 : return "[Final] No valid object for PPT";
            case 229 : return "[Final] Unknown color of container";
            case 230 : return "[Final] No valid picks left";
            case 231 : return "[Final] No tables0 : Can't generate table0 picks without table0";
            case 232 : return "[Final] No tables5 : Can't generate table5 picks without table5";
            case 233 : return "[Final] No tables10 : Can't generate table10 picks without table10";
            case 234 : return "[Final] No tables15 : Can't generate table15 picks without table15";
            case 235 : return "[Final] Picks from cavity plattforms are not implemented yet";
            default  : return "Unknown error";
    }
  }

        std::vector<std::string> mTableMapping;
        std::vector<unsigned int> mJTables;
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

        static const size_t blue = atwork_commander_msgs::Object::CONTAINER_BLUE;
        static const size_t red = atwork_commander_msgs::Object::CONTAINER_RED;

        static const size_t tables0_id = 0;
        static const size_t tables5_id = 1;
        static const size_t tables10_id = 2;
        static const size_t tables15_id = 3;
        static const size_t conveyors_id = 4;
        static const size_t ppts_id = 5;
        static const size_t shelfs_id = 6;
        using Parameters = std::map<std::string, int>;
        Parameters paramBNT;
        Parameters paramBTT3;
        Parameters paramFinal;
        int estimatet_active;
        bool paramContainerInShelf;
        bool paramContainerOnPpt;
        bool paramContainerOnTurntable;
        bool paramFlexibleHeight;
        std::vector<std::vector<size_t>> validpicks;
        std::vector<size_t> picksleft;
        std::unordered_map<size_t, size_t> mTableTypes;
        vector<set<ObjectType*>> mAvailableObjectsPerTable;

        size_t tabletypes;
  using run = vector<array<int, 5>>;

  void readParameters(const string& taskName) {
    unsigned int id = 0;
    mTables0.clear();
    mTables5.clear();
    mTables10.clear();
    mTables15.clear();
    mConveyors.clear();
    mJTables.clear();
    mPpts.clear();
    mShelfs.clear();
    paramFinal.clear();
    mObjects.clear();
    mPptObjects.clear();
    mTablesInverse.clear();
    const auto& allowedTables = mTasks[taskName].allowedTables;
    mTablesInverse.push_back("");
    const auto& nTables = mTasks[taskName].normalTableTypes;

    // Copy Tables to mutable vector
    vector<Table> tables(mTables.size());
    transform(mTables.begin(), mTables.end(), tables.begin(),
      [](const pair<string, Table>& e){ return e.second; }
    );


    // shuffle vector

    //shuffle(tables.begin(), tables.end(), mRand);


    for( const Table t : tables ) {

      if( ! allowedTables.empty() &&
          ! count( allowedTables.begin(), allowedTables.end(), t.name ) )
        continue;

      if(t.type == "TT"                                &&
         mTasks[taskName].parameters["tt_grasping"]==0 &&
         mTasks[taskName].parameters["tt_placing"]==0) {
        ROS_DEBUG_STREAM("Skipping turntable " << id);
        continue;
      }

      if(t.type == "PP"                                &&
         mTasks[taskName].parameters["pp_placing"]==0) {
        ROS_DEBUG_STREAM("Skipping precision placement table " << id);
        continue;
      }

      if(t.type == "SH"                                &&
         mTasks[taskName].parameters["shelfes_grasping"]==0 &&
         mTasks[taskName].parameters["shelfes_placing"]==0) {
        ROS_DEBUG_STREAM("Skipping shelf " << id);
        continue;
      }

      if(t.type != "TT" && t.type!= "PP" && t.type!= "SH" &&
         count(nTables.begin(), nTables.end(), t.type)==0) {
        ROS_DEBUG_STREAM("Skipping unwanted height " << t.type << " table " << id);
        continue;
      }

      if( t.type == "00" )                  { mTables0.push_back(++id);   mTablesInverse.push_back(t.name); continue; }
      if( t.type == "05" )                  { mTables5.push_back(++id);   mTablesInverse.push_back(t.name); continue; }
      if( t.type == "10" )                  { mTables10.push_back(++id);  mTablesInverse.push_back(t.name); continue; }
      if( t.type == "15" )                  { mTables15.push_back(++id);  mTablesInverse.push_back(t.name); continue; }
      if( t.type == "TT" || t.type == "CB") { mConveyors.push_back(++id); mTablesInverse.push_back(t.name); continue; }
      if( t.type == "PP" )                  { mPpts.push_back(++id);      mTablesInverse.push_back(t.name); continue; }
      if( t.type == "SH" )                  { mShelfs.push_back(++id);    mTablesInverse.push_back(t.name); continue; }

      ROS_ERROR_STREAM("Unknown table type" << t);
    }


    mJTables.resize(mTables0.size()+mTables5.size()+mTables10.size()+mTables15.size());
    auto it = mJTables.begin();
    it = copy(mTables0.begin(),  mTables0.end(),  it);
    it = copy(mTables5.begin(),  mTables5.end(),  it);
    it = copy(mTables10.begin(), mTables10.end(), it);
    it = copy(mTables15.begin(), mTables15.end(), it);


    ROS_DEBUG_STREAM("Allowed Tables : " << allowedTables);
    ROS_DEBUG_STREAM("Normal Tables  : " << mJTables);
    ROS_DEBUG_STREAM("0cm Tables     : " << mTables0);
    ROS_DEBUG_STREAM("5cm Tables     : " << mTables5);
    ROS_DEBUG_STREAM("10cm Tables    : " << mTables10);
    ROS_DEBUG_STREAM("15cm Tables    : " << mTables15);
    ROS_DEBUG_STREAM("Conveyors      : " << mConveyors);
    ROS_DEBUG_STREAM("PPTs           : " << mPpts);
    ROS_DEBUG_STREAM("Shelfs         : " << mShelfs);

    TaskDefinition& taskParams = mTasks[taskName];

   for ( const pair<string, int>& e : taskParams.objects) {
     ObjectType obj(e.first, e.second);
     atwork_commander_msgs::Object objType = toTaskObject(obj);
     mObjects.push_back(objType.object);
   }
   ROS_DEBUG_STREAM("Objects       : " << mObjects);
   for(const ObjectType& o: mAvailableObjects) {
    bool found = false;
    for(const ObjectType& c: mAvailableCavities)
      if (o.form == c.form) { found = true; break; }
    for(const string& c: mTasks[taskName].cavities) {
      ObjectType cT(c, 1);
      if (o.form == cT.form) { found = false; break; }
    }
      if( found ) mPptObjects.push_back(toTaskObject(ObjectType(o)).object);
   }
   paramFinal.emplace("pick_shelfs", mTasks[taskName].parameters["shelfes_grasping"]);
   paramFinal.emplace("place_shelfs", mTasks[taskName].parameters["shelfes_placing"]);
   paramFinal.emplace("objects", mTasks[taskName].parameters["objects"]);
   paramFinal.emplace("decoys", mTasks[taskName].parameters["decoys"]);
   paramFinal.emplace("container_placing_r", mTasks[taskName].parameters["container_placing_r"]);
   paramFinal.emplace("container_placing_b", mTasks[taskName].parameters["container_placing_b"]);
   paramFinal.emplace("paired_containers", mTasks[taskName].parameters["paired_containers"]);
   auto rContNumIt = mConfig.arena().objects.find("CONTAINER_RED");
   paramFinal.emplace("CONTAINER_RED", rContNumIt!=mConfig.arena().objects.end()?rContNumIt->second:0);
   auto bContNumIt = mConfig.arena().objects.find("CONTAINER_BLUE");
   paramFinal.emplace("CONTAINER_BLUE", bContNumIt!=mConfig.arena().objects.end()?bContNumIt->second:0);
   paramFinal.emplace("place_cavity_container", 0);
   paramFinal.emplace("pick_turntables", mTasks[taskName].parameters["tt_grasping"]);
   paramFinal.emplace("place_turntables", mTasks[taskName].parameters["tt_placing"]);
   unsigned int tableObjects = mTasks[taskName].parameters["objects"]-mTasks[taskName].parameters["shelfes_grasping"] - mTasks[taskName].parameters["tt_grasping"];
   unsigned int allocated = mTasks[taskName].parameters["shelfes_grasping"] + mTasks[taskName].parameters["tt_grasping"];
   if( count(nTables.begin(), nTables.end(), "00") && ! mTables0.empty() ){
    unsigned int num = tableObjects / mTasks[taskName].normalTableTypes.size();
    allocated+=num;
    paramFinal.emplace("pick_tables0", num);
   }
   if( count(nTables.begin(), nTables.end(), "05") && ! mTables5.empty() ) {
    unsigned int num = tableObjects / mTasks[taskName].normalTableTypes.size();
    allocated+=num;
    paramFinal.emplace("pick_tables5", num);
   }
   if( count(nTables.begin(), nTables.end(), "15") && ! mTables15.empty() ) {
    unsigned int num = tableObjects / mTasks[taskName].normalTableTypes.size();
    allocated+=num;
    paramFinal.emplace("pick_tables15", num);
   }
   paramFinal.emplace("pick_tables10", mTasks[taskName].parameters["objects"]-allocated);
   paramFinal.emplace("pick_ppts", 0);
   paramFinal.emplace("place_ppts", mTasks[taskName].parameters["pp_placing"]);
   paramFinal.emplace("pick_cavity_plattforms", 0);
   paramFinal.emplace("FlexibleHeight", 0);


   ROS_DEBUG_STREAM("PPT Objects   : " << mPptObjects);
   ROS_DEBUG_STREAM("Parameters    : " << paramFinal);
  }

  size_t get_container_id(const size_t table, const size_t color) {
    // if there already is a container of this color on the same table
    for(size_t i=0; i<container_ids.size(); ++i) {
      if (container_ids.at(i).at(0) == table && container_ids.at(i).at(1) == color) {
        return container_ids.at(i).at(2);
      }
    }
    //else get new id from the worldmodel
    if(color == blue) {
      std::array<size_t, 3> id = {table, blue, contId++};
      container_ids.push_back(id);
      return id.at(2);
    }
    else if(color == red) {
      std::array<size_t, 3> id = {table, red, contId++};
      container_ids.push_back(id);
      return id.at(2);
    }
    else {throw 229;}
  }

  vector<array<int, 5>> fromTask(const Task& origTask) {
    Task task = origTask;
    vector<array<int,5>> run;
    readParameters(task.type);
    auto start  = Converter::toMap(task.arena_start_state);
    auto target = Converter::toMap(task.arena_target_state);
    auto immobile = Converter::intersect(start, target);
    auto startObjs = Converter::diff(start, immobile);
    auto targetObjs = Converter::diff(target, immobile);
    size_t i = 0;
    size_t tID = 0;
    container_ids.clear();
    contId=1;
    for(const auto& objs: immobile) {
      tID = find(mTablesInverse.begin(), mTablesInverse.end(), objs.first) - mTablesInverse.begin();
      for( const auto& o: objs.second ) {
        if(o.object == red || o.object == blue) {
          array<int, 5> t;
          t[0] = o.object;
          t[1] = tID;
          t[2] = -1;
          t[3] = -1;
          t[4] = ++i;
          run.push_back(t);
          get_container_id(tID, o.object);
        }
      }
    }
    for(const auto& objs: immobile) {
      tID = find(mTablesInverse.begin(), mTablesInverse.end(), objs.first) - mTablesInverse.begin();
      for( const auto& o: objs.second ) {
        if(!o.decoy){
          ROS_WARN_STREAM_COND_NAMED(o.object < atwork_commander_msgs::Object::CONTAINER_RED, "generator",
              "[REFBOX-Gen] Non moving object found, which is not a decoy:\n" << o);
          continue;
        }
        array<int, 5> t;
        t[0] = o.object;
        t[1] = tID;
        t[2] = -1;
        t[3] = -1;
        t[4] = ++i;
        run.push_back(t);
      }
    }
    for(const auto& objs: startObjs) {
      tID = find(mTablesInverse.begin(), mTablesInverse.end(), objs.first) - mTablesInverse.begin();
      for( const auto& o: objs.second ) {
        array<int, 5> t;
        t[0] = o.object;
        t[1] = tID;
        t[2] = find(mTablesInverse.begin(), mTablesInverse.end(), Converter::findObject(targetObjs, o, objs.first, true)) - mTablesInverse.begin();
        t[4] = ++i;
        switch(o.target) {
          default:
          case(atwork_commander_msgs::Object::EMPTY): t[3] = -1;break;
          case(red):
          case(blue): t[3] = get_container_id(tID, o.target); break;
        }
        run.push_back(t);
      }
    }

    return run;
  }

  void debugAll(const string info, const run &tasks) {
    ROS_DEBUG_STREAM("mAllTables "<<mAllTables<<endl<<
                     "validpicks "<<validpicks<<endl<<
                     "picksleft "<<picksleft<<endl<<
                     "tabletypes "<<tabletypes<<endl<<
                     "container_ids "<<container_ids<<endl<<
                     "tasks "<<tasks);
  }

  void debug_tasks(const string info, const run &tasks) const {
    ostringstream os;
    for(size_t i=0; i<tasks.size(); ++i) {
      os << tasks.at(i).at(obj_id)<<" "<<tasks.at(i).at(src_id)<<" "<<tasks.at(i).at(dst_id)<<" "<<tasks.at(i).at(cont_id)<<" "<<tasks.at(i).at(obj_id_id) << endl;
    ROS_DEBUG_STREAM(os.str());
    }
  }

  /* obj, src, dst, cont
   * BNT: location, orientation [0,1,2,3], time [ca. 1-5s]
   * objects is the number of tasks
   */

  // for every task set random object type
  // if there are already entries select ppt objects only
  void generate_objects(run &tasks) {
    const size_t count = tasks.size();
    //srand(time(NULL));
    for(size_t i=0; i<count; ++i) {
      if(tasks.at(i).at(obj_id) == -1) {
        if(tasks.at(i).at(dst_id) != -1) {
          auto& dstObjs = mAvailableObjectsPerTable.at(tasks.at(i).at(dst_id));
          auto& srcObjs = mAvailableObjectsPerTable.at(tasks.at(i).at(src_id));
          ROS_DEBUG_STREAM("\tSrc(" << mTablesInverse.at(tasks.at(i).at(src_id)) << "): " << srcObjs << endl <<
                           "\tDst(" << mTablesInverse.at(tasks.at(i).at(dst_id)) << "): " << dstObjs);
          set<ObjectType*> objs;
          set_intersection(srcObjs.begin(), srcObjs.end(), dstObjs.begin(), dstObjs.end(), inserter(objs, objs.begin()));
          if(objs.size()==0) {
            ROS_ERROR_STREAM("No possible objects left: " << endl <<
                             "\tSrc(" << mTablesInverse.at(tasks.at(i).at(src_id)) << "): " << srcObjs << endl <<
                             "\tDst(" << mTablesInverse.at(tasks.at(i).at(dst_id)) << "): " << dstObjs);
            throw std::runtime_error("No possible objects left: ");
          }
          size_t objIndex = rand() % objs.size();
          ObjectType* obj;
          auto it = objs.begin();
          do
            obj=*it++;
          while(objIndex--!=0);
          tasks.at(i).at(obj_id) = toTaskObject(*obj).object;
          srcObjs.erase(obj);
          dstObjs.erase(obj);
          (*obj)--;
        }
        else {
          auto& srcObjs = mAvailableObjectsPerTable.at(tasks.at(i).at(src_id));
          if(srcObjs.size()==0) {
            ROS_ERROR_STREAM("No possible objects left: " << endl <<
                             "\tSrc(" << mTablesInverse.at(tasks.at(i).at(src_id)) << "): " << srcObjs);
            throw std::runtime_error("No possible objects left: ");
          }
          size_t objIndex = rand() % srcObjs.size();
          ObjectType* obj;
          auto it = srcObjs.begin();
          do
            obj=*it++;
          while(objIndex--!=0);
          tasks.at(i).at(obj_id) = toTaskObject(*obj).object;
          srcObjs.erase(obj);
          (*obj)--;
        }
      }

      for(auto& objects : mAvailableObjectsPerTable){
        std::set<ObjectType*> temp;
        copy_if(objects.begin(), objects.end(), inserter(temp, temp.begin()), [](ObjectType* o){ return (bool)*o;});
        objects = temp;
      }
    }
  }

  void checkObjectTypes(const Task& task) const {
    auto checkFunc =
    [](const atwork_commander_msgs::Object& obj){
      if(obj.object == atwork_commander_msgs::Object::EMPTY)
        throw runtime_error("Invalid object type in task: EMPTY");
      if(obj.object >= atwork_commander_msgs::Object::CONTAINER_RED && obj.decoy)
        throw runtime_error("Invalid object combination: Container or Cavity ("+to_string(obj.object)+") cannot be decoy");
      if(obj.target != atwork_commander_msgs::Object::EMPTY && obj.target < atwork_commander_msgs::Object::CONTAINER_RED)
        throw runtime_error("Invalid object combination: Target ("+to_string(obj.target)+") of object ("+to_string(obj.object)+")not container or cavity:");
      if(obj.target != atwork_commander_msgs::Object::EMPTY && obj.decoy)
        throw runtime_error("Invalid object combination: Decoys ("+to_string(obj.object)+") cannot have targets");
    };
    auto checkCont = [](const atwork_commander_msgs::Object& o){ return o.target == atwork_commander_msgs::Object::EMPTY;};
    auto checkCavity = [](const atwork_commander_msgs::Object& o){ return o.object < atwork_commander_msgs::Object::F20_20_H;};
    auto checkCavityTarget = [this](const atwork_commander_msgs::Object& o){
      if(o.object >= atwork_commander_msgs::Object::F20_20_H) return true;
      if(o.target == toCavity(o.object, Orientation::HORIZONTAL)) return true;
      if(o.target == toCavity(o.object, Orientation::VERTICAL)) return true;
      if(o.target == toCavity(o.object, Orientation::FREE)) return true;
      throw runtime_error("Object ("+to_string(o.object)+") with invalid PPT-Cavity ("+to_string(o.target)+") to be placed on PPT");
    };
    auto checkPPT = [this](const atwork_commander_msgs::Workstation& ws){
      auto compFunc = [&ws](const pair<string, Table>& t){ return t.second.name == ws.name;};
      return any_of(mTables.equal_range("PP").first, mTables.equal_range("PP").second, compFunc);
    };

    for(const atwork_commander_msgs::Workstation& ws: task.arena_start_state) {
      try {
         for(const atwork_commander_msgs::Object& obj: ws.objects)
          checkFunc(obj);

        if( checkPPT(ws) && any_of(ws.objects.begin(), ws.objects.end(), checkCavity)) {
            ostringstream os;
            os << "Non-Cavity objects on PPT table ( " << ws.name << "):\n\t";
            for(const atwork_commander_msgs::Object& obj: ws.objects)
              if(checkCavity(obj))
                os << obj.object << " ";
            throw runtime_error(os.str());
          }
      } catch(const runtime_error& e) {
        throw runtime_error("Workstation "+ws.name+": "+e.what());
      }
    }

    for(const atwork_commander_msgs::Workstation& ws: task.arena_target_state) {
      try {
        for(const atwork_commander_msgs::Object& obj: ws.objects) {
          checkFunc(obj);

          auto checkTarget = [&obj](const atwork_commander_msgs::Object& o){return o.target != obj.target;};
          if(obj.target != atwork_commander_msgs::Object::EMPTY &&
             all_of(ws.objects.begin(), ws.objects.end(), checkTarget))
            throw runtime_error("No appropriate container or cavity found for object with target");
        }

        if( checkPPT(ws) )
          all_of(ws.objects.begin(), ws.objects.end(), checkCavityTarget);
      } catch(const runtime_error& e) {
        throw runtime_error("Workstation "+ws.name+": "+e.what());
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
    if(k > position.size()) {
      throw std::logic_error("Can't choose " + std::to_string(k) + " from " + std::to_string(position.size()) +  " objects!");
    }
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

  std::vector<size_t> findValidContainerTables() {
    size_t size = mTables0.size() + mTables5.size() + mTables10.size()
            + mTables15.size();
    if(paramContainerOnTurntable == true) {
      size += mConveyors.size();
    }
    if(paramContainerOnPpt == true) {
      size += mPpts.size();
    }
    if(paramContainerInShelf == true) {
      size += mShelfs.size();
    }
    std::vector<size_t> valid_containerTables(size);
    auto end = copy(mTables0.begin(), mTables0.end(), valid_containerTables.begin());
    end = copy(mTables5.begin(), mTables5.end(), end);
    end = copy(mTables10.begin(), mTables10.end(), end);
    end = copy(mTables15.begin(), mTables15.end(), end);
    if(paramContainerOnTurntable == true) {
      end = copy(mConveyors.begin(), mConveyors.end(), end);
    }
    if(paramContainerOnPpt) {
      end = copy(mPpts.begin(), mPpts.end(), end);
    }
    if(paramContainerInShelf == true) {
      end = copy(mShelfs.begin(), mShelfs.end(), end);
    }
    return valid_containerTables;
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
    picksleft.at(shelfs_id) = paramFinal["pick_shelfs"];
  }


  void initialize_mTableTypes() {
    for(size_t table : mTables0) {
      ROS_DEBUG_STREAM("Entering 0cm table " << table << " in mTableTypes");
      mTableTypes[table] = tables0_id;
    }
    for(size_t table : mTables5) {
      ROS_DEBUG_STREAM("Entering 5cm table " << table << " in mTableTypes");
      mTableTypes[table] = tables5_id;
    }
    for(size_t table : mTables10) {
      ROS_DEBUG_STREAM("Entering 10cm table " << table << " in mTableTypes");
      mTableTypes[table] = tables10_id;
    }
    for(size_t table : mTables15) {
      ROS_DEBUG_STREAM("Entering 15cm table " << table << " in mTableTypes");
      mTableTypes[table] = tables15_id;
    }
    for(size_t table : mConveyors) {
      ROS_DEBUG_STREAM("Entering tt table " << table << " in mTableTypes");
      mTableTypes[table] = conveyors_id;
    }
    for(size_t table : mPpts) {
      ROS_DEBUG_STREAM("Entering pp table " << table << " in mTableTypes");
      mTableTypes[table] = ppts_id;
    }
    for(size_t table : mShelfs) {
      ROS_DEBUG_STREAM("Entering shelf table " << table << " in mTableTypes");
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
    size_t minindex=0;													// define minindex
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

  Task generateImpl( const string& taskName){
    readParameters(taskName);
    try {
      // initialize tasks
      run tasks(paramFinal["objects"], {-1, -1, -1, -1, -1});

      unsigned int maxNTables = mTasks[taskName].parameters["tables"];

      if(maxNTables != 0 && mTasks[taskName].allowedTables.empty()) {

        const std::vector<std::vector<unsigned int>*> tables({&mTables0, &mTables5, &mTables10, &mTables15, &mConveyors, &mPpts, &mShelfs});
        std::vector<std::vector<unsigned int>*> workingTables(tables);

        for(const auto vPtr: tables)
          shuffle(vPtr->begin(), vPtr->end(), mRand);

        auto sumUp=[](std::vector<std::vector<unsigned int>*> tables){
          unsigned int sum=0;
          for(const vector<unsigned int>* vPtr: tables)
            sum+=vPtr->size();
          return sum;
        };

        while(sumUp(tables) > maxNTables) {

          workingTables.erase(
            remove_if(workingTables.begin(), workingTables.end(),
                      [](std::vector<unsigned int>* v){ return v->size()<=1; }
                     ),
            workingTables.end()
          );

          if(workingTables.empty())
            throw std::runtime_error("Not enough tables requested: More table types in task then tables requested!");

          shuffle(workingTables.begin(), workingTables.end(), mRand);
          unsigned int d=min((unsigned int)workingTables.size(), sumUp(tables)-maxNTables);
          ROS_DEBUG_STREAM("Removing tables from " << d << "types");

          for(unsigned int i=0; i<d; i++) {
            ROS_DEBUG_STREAM("Removing table " << workingTables.at(i)->at(workingTables.at(i)->size()-1));
            workingTables.at(i)->erase(workingTables.at(i)->end()-1,workingTables.at(i)->end());
          }
        }
      }
      {
        mJTables.clear();
        mJTables.resize(mTables0.size()+mTables5.size()+mTables10.size()+mTables15.size());
        auto it = mJTables.begin();
        it = copy(mTables0.begin(),  mTables0.end(),  it);
        it = copy(mTables5.begin(),  mTables5.end(),  it);
        it = copy(mTables10.begin(), mTables10.end(), it);
        it = copy(mTables15.begin(), mTables15.end(), it);
      }

      ROS_DEBUG_STREAM("Task specific Tables : " << endl <<
                       "Allowed Tables : " << mTasks[taskName].allowedTables << endl <<
                       "Normal Tables  : " << mJTables << endl <<
                       "0cm Tables     : " << mTables0 << endl <<
                       "5cm Tables     : " << mTables5 << endl <<
                       "10cm Tables    : " << mTables10 << endl <<
                       "15cm Tables    : " << mTables15 << endl <<
                       "Conveyors      : " << mConveyors << endl <<
                       "PPTs           : " << mPpts << endl <<
                       "Shelfs         : " << mShelfs);
      size_t shelfs = mShelfs.size();
      size_t tables = mJTables.size();
      size_t ppts = mPpts.size();
      size_t conveyors = mConveyors.size();
      size_t table0 = mTables0.size();
      size_t table5 = mTables5.size();
      size_t table10 = mTables10.size();
      size_t table15 = mTables15.size();
      mAvailableObjectsPerTable = vector<set<ObjectType*>>(mTablesInverse.size());
      container_ids.clear();
      contId=1;
      for(size_t i =1; i<mTablesInverse.size(); i++) {
        auto checkPPT = [this](const string& name){
          auto compFunc = [&name](const pair<string, Table>& t){ return t.second.name == name;};
          return any_of(mTables.equal_range("PP").first, mTables.equal_range("PP").second, compFunc);
        };
        set<ObjectType> temp;
        for(ObjectType& obj: mAvailableObjects) {
          try{
            auto taskObj = toTaskObject(obj).object;
            if( taskObj == atwork_commander_msgs::Object::EMPTY) continue;
            if( taskObj >= atwork_commander_msgs::Object::CONTAINER_RED) continue;
            if( checkPPT(mTablesInverse[i]) && taskObj >=  atwork_commander_msgs::Object::ADVANCED_START ) continue;
            mAvailableObjectsPerTable[i].insert(&obj);
          }catch(...){
            ROS_ERROR_STREAM("Object lost because of matching error: " << endl << obj);
            continue;
          }
        }
      }

      if(paramFinal["FlexibleHeight"] == true) {
        tabletypes = 8;
      } else {
        tabletypes = 7;
      }
      ROS_DEBUG_STREAM("JTables: [ "<<mJTables << " ]");
      // check validity
      if (mObjects.size() == 0) {throw 220;}																			                               // No Objects
      if (shelfs == 0 && (paramFinal["pick_shelfs"] > 0 || paramFinal["place_shelfs"] > 0)) {throw 221;}				   // No shelfpick/place without shelf
      if (tables == 0 && paramFinal["objects"] > paramFinal["pick_shelfs"]) {throw 222;}								           // No tables
      if (shelfs + tables + ppts +conveyors + paramFinal["B_Container"] + paramFinal["R_Container"] <= 1) {throw 223;} // pick = place
      if (paramContainerInShelf == true) {
        if (tables + shelfs == 0 && paramFinal["B_Container"] + paramFinal["R_Container"] > 0) {throw 224;}			 // No tables, no shelfs, no conveyors, No container place in air
      }
      else if(tables == 0 && paramFinal["B_Container"] + paramFinal["R_Container"] > 0) {throw 225;}					   // No tables No container place in air
      if (ppts == 0 && paramFinal["place_ppts"] > 0) {throw 226;}										                             // No cavity plattforms
      if (conveyors == 0 && (paramFinal["pick_turntables"] > 0 || paramFinal["place_turntables"] > 0)) {throw 227;}	// No conveyors
      if (table0 == 0 && paramFinal["pick_tables0"] > 0) {throw 231;}													// No tables0
      if (table5 == 0 && paramFinal["pick_tables5"] > 0) {throw 232;}													// No tables5
      if (table10 == 0 && paramFinal["pick_tables10"] > 0) {throw 233;}												// No tables10
      if (table15 == 0 && paramFinal["pick_tables15"] > 0) {throw 234;}												// No tables15
      if (paramFinal["pick_cavity_plattforms"] > 0) {throw 235;}														  // Picks from cavity plattform are not implemented yet

      /* select #place_shelfs + #place_turntables + #ppts random tasks
       * select #place_shelfs + #place_turntables of these
       * the rest of the places are on a random turntable
       * select #place_shelfs of these and set place to a random shelf
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
      std::vector<size_t>containerTables;
      size_t specialplaces = paramFinal["place_shelfs"] + paramFinal["place_turntables"] + paramFinal["place_ppts"];
      size_t shelfTurntables = paramFinal["place_shelfs"] + paramFinal["place_turntables"];
      size_t a;
      variation(position, specialplaces, specialplace, normalplace);
      variation(specialplace, shelfTurntables ,placeShelfTurntable, placePpt);
      variation(placeShelfTurntable, paramFinal["place_shelfs"], placeShelf, placeTurntable);

      unsigned int contPlace = paramFinal["container_placing_b"] + paramFinal["container_placing_r"];
      if(contPlace > 0) {
        // collect all valid tables to place containers
        std::vector<size_t> valid_containerTables = findValidContainerTables();
        unsigned int rContNum = paramFinal["container_placing_r"];
        unsigned int bContNum = paramFinal["container_placing_b"];

        size_t numberOfContainerTables;
        if(paramFinal["paired_containers"] == true) {
          numberOfContainerTables = std::min(rContNum, bContNum);
        } else {
          numberOfContainerTables = rContNum + bContNum;
        }
        // choose the tables with containers
        variation(valid_containerTables, numberOfContainerTables, containerTables);

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

        // choose randomly the task with containerplaces
        variation(valid_containerplaces, contPlace, container);
        variation(container, bContNum, b_container, r_container);

        for(size_t i=0; i<contPlace; ++i) {
          a = rand() % valid_containerTables.size();
          tasks.at(container.at(i)).at(dst_id) = valid_containerTables.at(a);
        }

        for(auto& container: b_container) {
          size_t table = tasks.at(container).at(dst_id);
          size_t blue_container_id = get_container_id(table, blue);
          tasks.at(container).at(cont_id) = blue_container_id;
          if(paramFinal["paired_containers"] == true) {
            get_container_id(table,red);
          }
        }
        for(auto& container: r_container) {
          size_t table = tasks.at(container).at(dst_id);
          size_t red_container_id = get_container_id(table, red);
          tasks.at(container).at(cont_id) = red_container_id;
          if(paramFinal["paired_containers"] == true) {
            get_container_id(table,blue);
          }
        }
      }

      // write the Ppts as destinations to the tasks
      for(size_t i=0; i<placePpt.size(); ++i) {
        if(tasks.at(placePpt.at(i)).at(dst_id) == -1) {
          a = rand() % ppts;
          tasks.at(placePpt.at(i)).at(dst_id) = mPpts.at(a);
        }
      }

      // write all the other distinations to the tasks
      for(size_t i=0; i<placeShelf.size(); ++i) {
        if(tasks.at(placeShelf.at(i)).at(dst_id) == -1) {
          a = rand() % shelfs;
          tasks.at(placeShelf.at(i)).at(dst_id) = mShelfs.at(a);
        }
      }
      for(size_t i=0; i<placeTurntable.size(); ++i) {
        if(tasks.at(placeTurntable.at(i)).at(dst_id) == -1) {
          a = rand() % conveyors;
          tasks.at(placeTurntable.at(i)).at(dst_id) = mConveyors.at(a);
        }
      }

      // BMT
      if(taskName == "BMT") {
        a = rand() % tables;
        for(size_t i=0; i<normalplace.size(); ++i) {
          tasks.at(normalplace.at(i)).at(dst_id) = mJTables.at(a);
        }
      } else {
        for(size_t i=0; i<normalplace.size(); ++i) {
          if(tasks.at(normalplace.at(i)).at(dst_id) == -1) {
            a = rand() % tables;
            tasks.at(normalplace.at(i)).at(dst_id) = mJTables.at(a);
          }
        }
      }

      initialize_mAllTables();
      initialize_picksleft();
      initialize_mTableTypes();
      initialize_validpicks(tasks);										                          // generate a list of valid picks for each task

      while(sum_vector(picksleft) > 0) {
        size_t min = numeric_limits<size_t>::max();						                  // set min to the mximum value of size_t;
        size_t index = shortest_list(min);								                      // min is a refference
        if (min == numeric_limits<size_t>::max()) {throw 230;}			            // if all validpick vectors are empty
        a = rand() % min;												                                // select a random valid pick for these table
        size_t table = validpicks.at(index).at(a);					                   	// set table to the ID for these table
        tasks.at(index).at(src_id) = table;
        validpicks.at(index).resize(0);								                         	// task has table => no validpicks left
        size_t type_id = mTableTypes.at(table);				                     			// set type to the type of table (mTableTypes is a map)
        --picksleft.at(type_id);									                             	// reduce the number of picks left
        update_validpicks(type_id);								                           		// update the lists of vaild picks
      }

      decltype(tasks) decoyTasks(paramFinal["decoys"]);
      auto it  = decoyTasks.begin();
      for(const auto& task: tasks) {
        if(it == decoyTasks.end())
          break;
        if(mTableTypes[task[src_id]] ==shelfs_id || mTableTypes[task[src_id]] == conveyors_id )
          (*it++)={-1, task[src_id], -1, -1, -1};
      }
      while(it != decoyTasks.end())
        for(const auto& task: tasks) {
          if(mTableTypes[task[src_id]] !=shelfs_id && mTableTypes[task[src_id]] != conveyors_id )
            (*it++)={-1, task[src_id], -1, -1, -1};
          if(it == decoyTasks.end())
            break;
        }

      tasks.resize(tasks.size()+decoyTasks.size());
      copy(decoyTasks.begin(), decoyTasks.end(), tasks.end()-decoyTasks.size());

      generate_objects(tasks);                                                  // generate objects for the decoys
      debugAll("taskgenerierung", tasks);
      checkPickNeqPlace(tasks);
      checkPickCounts(tasks, paramFinal);
      checkPlaceCounts(tasks, paramFinal);
      const_cast<Generator*>(this)->checkContainers(tasks, paramFinal);
      Task task = toTask(tasks);
      vector<uint16_t> remainingCavities(mAvailableCavities.size());
      transform(mAvailableCavities.begin(), mAvailableCavities.end(), remainingCavities.begin(), &Generator::toCavityType);
      auto cavitiesEnd = remainingCavities.end();
      for(atwork_commander_msgs::Workstation& ws: task.arena_target_state) {
        auto checkPPT = [this](const atwork_commander_msgs::Workstation& ws){
          auto compFunc = [&ws](const pair<string, Table>& t){ return t.second.name == ws.name;};
          return any_of(mTables.equal_range("PP").first, mTables.equal_range("PP").second, compFunc);
        };
        if(checkPPT(ws)) {
          vector<atwork_commander_msgs::Object> cavities;
          for(atwork_commander_msgs::Object& o: ws.objects) {
            if(o.decoy) continue;
            if(o.target<atwork_commander_msgs::Object::CAVITY_START ||
               o.target>=atwork_commander_msgs::Object::CAVITY_END) continue;
            cavitiesEnd = remove(cavitiesEnd, remainingCavities.end(), o.target);
            atwork_commander_msgs::Object cavity;
            cavity.object = o.target;
            cavities.push_back(cavity);
          }
          sort(cavities.begin(), cavities.end(), [](const atwork_commander_msgs::Object& a, const atwork_commander_msgs::Object& b){ return a.object < b.object; });
          auto rem = unique(cavities.begin(), cavities.end());
          cavities.erase(rem, cavities.end());
          while(cavities.size() < 5){
            if(cavitiesEnd == remainingCavities.begin())
              throw runtime_error("Not enough cavities left");
            size_t i = rand() %  (cavitiesEnd - remainingCavities.begin());
            atwork_commander_msgs::Object cavity;
            cavity.object = remainingCavities[i];
            cavities.push_back(cavity);
            iter_swap(remainingCavities.begin()+i, cavitiesEnd);
            cavitiesEnd--;
          }
          shuffle(cavities.begin(), cavities.end(), mt19937(random_device()()));
          copy(cavities.begin(), cavities.end(), back_inserter(ws.objects));
          auto it = find_if(task.arena_start_state.begin(), task.arena_start_state.end(), [&ws](const atwork_commander_msgs::Workstation& startWS){return startWS.name == ws.name;});
          if(it == task.arena_start_state.end()) throw runtime_error("PPT does not exist in start state: "+ws.name);
          copy(cavities.begin(), cavities.end(), back_inserter(it->objects));
        }
      }
      return task;
    } catch(int i) {
      throw runtime_error(printError(i));
    }
    catch(const string& e) {
      throw runtime_error(e);
    }
  }

  public:
    void checkPickNeqPlace(const run& tasks) const {
      for(size_t i=0; i<tasks.size(); ++i) {
        if(tasks.at(i).at(dst_id) == tasks.at(i).at(src_id) && tasks.at(i).at(cont_id) == -1) { // if place in container pick = place is valid
          std::string errormessage = "Task " + to_string(i);
          errormessage += " violates the rule pick != place.\n";
          throw errormessage;
        }
      }
      ROS_DEBUG_STREAM_NAMED("generator.check", "All tasks satisfy the rule pick != place.");
    }

    void checkPickCounts(const run& tasks, Parameters params) const {
      std::vector<size_t> counter(tabletypes,0);
      for(size_t i=0; i<tasks.size(); ++i) {
        if(tasks.at(i).at(dst_id) != -1) {                                      // don't count decoys
          counter.at(mTableTypes.at(tasks.at(i).at(src_id)))++;
        }
      }
      if(counter.at(tables0_id) != size_t(params["pick_tables0"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_tables0"]));
        errormessage += " wanted, but " + to_string(counter.at(tables0_id)) + " pick_tables0 found.\n";
        throw errormessage;
      }
      if(counter.at(tables5_id) != size_t(params["pick_tables5"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_tables5"]));
        errormessage += " wanted, but " + to_string(counter.at(tables5_id)) + " pick_tables5 found.\n";
        throw errormessage;
      }
      if(counter.at(tables10_id) != size_t(params["pick_tables10"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_tables10"]));
        errormessage += " wanted, but " + to_string(counter.at(tables10_id)) + " pick_tables10 found.\n";
        throw errormessage;
      }
      if(counter.at(tables15_id) != size_t(params["pick_tables15"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_tables15"]));
        errormessage += " wanted, but " + to_string(counter.at(tables15_id)) + " pick_tables15 found.\n";
        throw errormessage;
      }
      if(counter.at(conveyors_id) != size_t(params["pick_turntables"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_turntables"]));
        errormessage += " wanted, but " + to_string(counter.at(conveyors_id)) + " pick_turntables found.\n";
        throw errormessage;
      }
      if(counter.at(shelfs_id) != size_t(params["pick_shelfs"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_shelfs"]));
        errormessage += " wanted, but " + to_string(counter.at(shelfs_id)) + " pick_shelfs found.\n";
        throw errormessage;
      }
      if(counter.at(ppts_id) != size_t(params["pick_ppts"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["pick_ppts"]));
        errormessage += " wanted, but " + to_string(counter.at(ppts_id)) + " pick_ppts found.\n";
        throw errormessage;
      }
      ROS_DEBUG_STREAM_NAMED("generator.check", "All table types occure with correct multiplicities in picks.");
    }

    void checkPlaceCounts(const run& tasks, Parameters params) const {
      std::vector<size_t> counter(tabletypes,0);
      size_t decoys = 0;
      for(size_t i=0; i<tasks.size(); ++i) {
        if(tasks.at(i).at(obj_id) != red && tasks.at(i).at(obj_id) != blue) {
          if(tasks.at(i).at(dst_id) == -1) {                                      // count decoys
            ++decoys;
          } else {
            counter.at(mTableTypes.at(tasks.at(i).at(dst_id)))++;                 // not decoys
          }
        }
      }
      if(decoys != (size_t)params["decoys"]) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["decoys"]));
        errormessage += " wanted, but " + to_string(decoys) + " decoys found.\n";
        throw errormessage;
      }
      else {
        ROS_DEBUG_STREAM_NAMED("generator.check", "Decoys occure with correct multiplicities.");
      }
      if(counter.at(conveyors_id) != size_t(params["place_turntables"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["place_turntables"]));
        errormessage += " wanted, but " + to_string(counter.at(conveyors_id)) + " place_turntables found.\n";
        throw errormessage;
      }
      if(counter.at(shelfs_id) != size_t(params["place_shelfs"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["place_shelfs"]));
        errormessage += " wanted, but " + to_string(counter.at(shelfs_id)) + " place_shelfs found.\n";
        throw errormessage;
      }
      if(counter.at(ppts_id) != size_t(params["place_ppts"])) {
        std::string errormessage = "Missmatch: " + to_string(size_t(params["place_ppts"]));
        errormessage += " wanted, but " + to_string(counter.at(ppts_id)) + " place_ppts found.\n";
        throw errormessage;
      }
      size_t normalplaces_found = counter.at(tables0_id) + counter.at(tables5_id) + counter.at(tables10_id) + counter.at(tables15_id);
      size_t normalplaces_wanted = params["objects"] - params["place_turntables"] - params["place_shelfs"] - params["place_ppts"];
      if(normalplaces_found != normalplaces_wanted) {
        std::string errormessage = "Missmatch: " + to_string(normalplaces_wanted);
        errormessage += " wanted, but " + to_string(normalplaces_found) + " normalplaces found.\n";
        throw errormessage;
      }
      ROS_DEBUG_STREAM_NAMED("generator.check", "All table types occure with correct multiplicities in places.");
    }

    void checkContainers(const run& tasks, Parameters params) {
//      size_t bluecontainer = 0, redcontainer = 0;
//      for(size_t i=0; i<container_ids.size(); ++i) {
//        if(container_ids.at(i).at(1) == blue)
//          ++bluecontainer;
//        if(container_ids.at(i).at(1) == red)
//          ++redcontainer;
//      }
//      if(bluecontainer != (size_t)mConfig.arena().objects.at("CONTAINER_BLUE")) {
//        std::string errormessage = "Missmatch: " + to_string(mConfig.arena().objects.at("CONTAINER_BLUE"));
//        errormessage += " wated, but " + to_string(bluecontainer) + " blue containers found.\n";
//        throw errormessage;
//      }
//      if(redcontainer != (size_t)mConfig.arena().objects.at("CONTAINER_RED")) {
//        std::string errormessage = "Missmatch: " + to_string(mConfig.arena().objects.at("CONTAINER_RED"));
//        errormessage += " wated, but " + to_string(redcontainer) + " red containers found.\n";
//        throw errormessage;
//      }

      if(paramFinal["paired_containers"] == true) {
        vector<std::array<size_t, 3>> red_container_ids, blue_container_ids;
        for(size_t i=0; i<container_ids.size(); ++i) {
          if(container_ids.at(i).at(1) == blue) {
              blue_container_ids.push_back(container_ids.at(i));
          }
          else {
            red_container_ids.push_back(container_ids.at(i));
          }
        }
        auto comp = [](const auto& left, const auto& right){return left.at(0) < right.at(0);};
        std::sort(begin(blue_container_ids),end(blue_container_ids), comp);
        std::sort(begin(red_container_ids),end(red_container_ids), comp);
        ROS_DEBUG_STREAM("blue containers: " << blue_container_ids);
        ROS_DEBUG_STREAM("red containers: " << red_container_ids);
        if(blue_container_ids.size() != red_container_ids.size())
          throw runtime_error("Different amounts of containers created: BLUE: "+to_string(blue_container_ids.size())+"vs. RED: "+to_string(red_container_ids.size())+"!");
        for(size_t i=0; i<blue_container_ids.size(); ++i) {
          if(blue_container_ids.at(i).at(0) != red_container_ids.at(i).at(0)) {
            std::string errormessage = "There is only one color of containers on table with id " + to_string(i) +".";
            throw errormessage;
          }
        }
      }
      size_t bTarget = 0, rTarget = 0;
      for(const auto& task: tasks) {
        if(task.at(cont_id)!=-1) {
          switch(container_ids[task.at(cont_id)-1][1]){
            case(red): rTarget++;
                       break;
            case(blue):bTarget++;
          }
        }
      }
      if(bTarget != paramFinal["container_placing_b"]) {
          std::string errormessage = "There are not enough objects to be placed in BLUE CONTAINERs: ";
          errormessage += std::to_string(bTarget) + " <-> " + std::to_string(paramFinal["container_placing_b"]);
          throw errormessage;
      }
      if(rTarget != paramFinal["container_placing_r"]) {
          std::string errormessage = "There are not enough objects to be placed in RED CONTAINERs: ";
          errormessage += std::to_string(rTarget) + " <-> " + std::to_string(paramFinal["container_placing_r"]);
          throw errormessage;
      }

      ROS_DEBUG_STREAM_NAMED("generator", "All container types occure with correct multiplicities.");
    }

    virtual void onInit(const std::string& arenaConfig, const std::string& taskConfig)
    {
      try {
        mConfig.reload(arenaConfig, taskConfig);
        mAvailableCavities = extractCavities( mConfig.arena() );
        mAvailableObjects = extractObjectTypes( mConfig.arena() );
        for (const auto& table: mConfig.arena().workstations)
          mTables.emplace(table.second, Table(table.first, table.second));
        sanityCheck();
      }
      catch(const exception& e) {
        ROS_ERROR_STREAM_NAMED("generator", "[REFBOX-GEN] Error during initialization of plugin: " << e.what());
        throw;
      }
    }


    virtual bool check( const Task& task ) const {
      const run tasks = const_cast<Generator*>(this)->fromTask(task);
      try {
      debug_tasks("final tasks", tasks);
        checkObjectTypes(task);
        checkPickNeqPlace(tasks);
        checkPickCounts(tasks, paramFinal);
        checkPlaceCounts(tasks, paramFinal);
        const_cast<Generator*>(this)->checkContainers(tasks, paramFinal);
      } catch(std::string error) {
        ROS_ERROR_STREAM(error);
        return false;
      } catch(std::runtime_error& e) {
        ROS_ERROR_STREAM(e.what());
        return false;
//      } catch(std::exception& e) {
//        ROS_ERROR_STREAM(e.what());
//        return false;
//      }catch(...) {
//        ROS_ERROR_STREAM("Unknown error in checking task");
//        return false;
      }
      return true;
    }

    virtual Task generate(const std::string& taskName) {
      using Clock = chrono::steady_clock;
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
      task.type = taskName;
      auto creationTime = Clock::now().time_since_epoch();
      task.id = chrono::duration_cast<chrono::duration<decltype(task.id), std::milli>>(creationTime).count();
      return task;
    }

    virtual ConfigParserInterface& config() { return mConfig; }
};

}
}
}

PLUGINLIB_EXPORT_CLASS(atwork_commander::task_generator::jurek::Generator, atwork_commander::task_generator::GeneratorPluginInterface);

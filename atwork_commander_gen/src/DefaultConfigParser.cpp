#include <atwork_commander_gen/DefaultConfigParser.h>

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <stdexcept>

using namespace std;

template<typename T>
ostream& operator<<( ostream& os, const vector<T>& v) {
  os << "[";
  for ( const T& t : v )
    os << t << " ";
  return os << "]";
}

namespace atwork_commander {
namespace task_generator {

static bool readStrings(StringList& data, XmlRpc::XmlRpcValue& val) {
  if ( val.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
    ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Invalid Parameter type encountered! Expected Array!");
    return false;
  }

  data.resize( val.size() );
  auto it = data.begin();

  for (int i=0;i<val.size();i++)
    switch(val[i].getType()) {
      case( XmlRpc::XmlRpcValue::TypeString ) : *it++ =                 static_cast<std::string>( val[i] )  ; break;
      case( XmlRpc::XmlRpcValue::TypeInt )    : *it++ = std::to_string( static_cast<int>        ( val[i] ) ); break;
      case( XmlRpc::XmlRpcValue::TypeDouble ) : *it++ = std::to_string( static_cast<double>     ( val[i] ) ); break;
      case( XmlRpc::XmlRpcValue::TypeBoolean ): *it++ = std::to_string( static_cast<bool>       ( val[i] ) ); break;
      default:
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Invalid Parameter type encountered for parameter in array!" <<
                                                       " Allowed are String, Int, Bool, Double" );
        return false;
    };

  return true;
}

static bool readStruct(ParameterType& params, XmlRpc::XmlRpcValue& val) {
  if ( val.getType() != XmlRpc::XmlRpcValue::TypeStruct ) {
    ROS_WARN_STREAM_NAMED("state_tracked", "[REFBOX] Invalid Parameter type encountered for parameter!" <<
                                                   " Expected Struct!");
    return false;
  }
  bool result = true;
  for (auto& entry: val) {
    switch(entry.second.getType()) {
      case( XmlRpc::XmlRpcValue::TypeInt )    : params[entry.first]=static_cast<int>( entry.second ); break;
      case( XmlRpc::XmlRpcValue::TypeBoolean ): params[entry.first]=static_cast<bool>( entry.second ); break;
      default:
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Invalid Parameter type encountered for struct parameter!" <<
                                                       " Allowed are Int and Bool");
        result = false;
    };
  }
  return result;
}

static TaskDefinition readTask(const std::string& name, XmlRpc::XmlRpcValue& def ) {
  TaskDefinition task;

  for (auto& entry : def) {
    ROS_INFO_STREAM_NAMED("parser", "[REFBOX] Reading parameter: " << entry.first << " of task: " << name);
    if ( entry.first == "normal_table_types" ) {
      if ( !readStrings(task.normalTableTypes, entry.second) )
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading table types of normale tables for task "
                                               << name << "! Keeping default: " << task.normalTableTypes << "!");
      continue;
    }
    if ( entry.first == "allowed_tables" ) {
      if ( !readStrings(task.allowedTables, entry.second) )
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading list of explicitly allwoed tables for task "
                                               << name << "! Keeping default: " << task.allowedTables << "!");
      ROS_INFO_STREAM_NAMED("parser", "[REFBOX] Read allowed tables: " << task.allowedTables);
      continue;
    }
    if ( entry.first == "tt_types" ) {
      if ( !readStrings(task.ttTypes, entry.second) )
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading table types of turntables for task "
                                               << name << "! Keeping default: " << task.ttTypes << "!");
      continue;
    }
    if ( entry.first == "pp_types" ) {
      if ( !readStrings(task.ppTypes, entry.second) )
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading table types of precision placement for task " <<
                                               name <<  "! Keeping default: " << task.ppTypes << "!");
      continue;
    }
    if ( entry.first == "sh_types" ) {
      if ( !readStrings(task.shTypes, entry.second) )
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading table types of shelfs for task " << name <<
                                               "! Keeping default: " << task.shTypes << "!");
      continue;
    }
    if ( entry.first == "cavities" ) {
      if ( !readStrings(task.cavities, entry.second) )
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading available cavities for task " << name <<
                                               "! Keeping default: " << task.cavities << "!");
      continue;
    }
    if ( entry.first == "object_types" ) {
      if ( !readStruct(task.objects, entry.second) ) {
        std::ostringstream os;
        for ( const auto& v: task.objects )
          os << "\t" << v.first << ": " << v.second << std::endl;
        ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Error reading available objects for task " << name <<
                                               "! State after partial update: " << std::endl << os.str());

      }
      continue;
    }

    auto it = task.parameters.find(entry.first);
    if ( it != task.parameters.end()) {
      switch(entry.second.getType()) {
        case( XmlRpc::XmlRpcValue::TypeInt )    : it->second = static_cast<int>( entry.second ); break;
        case( XmlRpc::XmlRpcValue::TypeBoolean ): it->second = static_cast<bool>( entry.second ); break;
        default:
          ROS_ERROR_STREAM_NAMED("parser", "[REFBOX] Invalid Parameter type encountered for parameter " <<
                                                  name << "/" << entry.first << "! Only int and bool is allowed!");
      };
    } else {
      ROS_WARN_STREAM_NAMED("parser", "[REFBOX] Unknown parameter " << entry.first << " in definition of task " << name << "! Ignoring!");
    }
  }

  return task;
}

static TaskDefinitions readTaskList(const string& taskConfig) {
  TaskDefinitions tasks;
  ros::NodeHandle nh;
  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] Try to read task definitions from '" << taskConfig << "'");

  XmlRpc::XmlRpcValue my_list;
  try {
  nh.getParam(taskConfig, my_list);
  } catch(const ros::InvalidNameException& e) {
    ROS_ERROR_STREAM_NAMED("parser", "Invalid Name for task config specified: " << taskConfig << ": " << e.what());
    throw;
  }

  if (my_list.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM_NAMED("parser", "[REFBOX] Couldn't read Tasklist. Aspect 'XmlRpc::XmlRpcValue::TypeStruct' under " <<  taskConfig << "!");
      throw runtime_error(string("No struct under ") +taskConfig +"! bailing out!");
  }

  for (auto& task_p : my_list) {
      std::string name = static_cast<std::string>(task_p.first);

      if (task_p.second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          ROS_WARN_STREAM_NAMED("parser", "[REFBOX] " << taskConfig << "/" << name << " is not a task definition");
          continue;
      }

      tasks[name] = readTask(name, task_p.second);
  }

  ROS_ASSERT(tasks.size() > 0);
  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] Read " << tasks.size() << " tasks from paramet er server");
  ROS_DEBUG_STREAM_NAMED("parser", "[REFBOX] Defined Tasks: " << tasks);
  return tasks;
}

static ArenaDescription readArenaDefinition(const string& arenaConfig) {
  ArenaDescription arena;
  ros::NodeHandle nh;
  string ws_param = arenaConfig + "/workstations";
  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] try to read workstations from '" << ws_param << "'") ;

  string wp_param = arenaConfig + "/waypoints";
  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] try to read waypoints from '" << wp_param << "'") ;

  string obj_param = arenaConfig + "/objects";
  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] try to read available objects from '" << obj_param << "'") ;
  try {
    map<string, string> temp_ws;
    if( ! nh.getParam(ws_param, temp_ws) ) throw runtime_error("No workstations defined for arena: "+arenaConfig);
    for (auto& ws : temp_ws) {
        arena.workstations[ws.first] = ws.second;
    }

    std::map<std::string, std::string> temp_wp;
    if( nh.getParam(wp_param, temp_wp) ) {
      for (auto& wp : temp_wp) {
          arena.waypoints[wp.first] = wp.second;
      }
    }

    std::map<std::string, int> temp_obj;
    if( ! nh.getParam(obj_param, temp_obj) ) throw runtime_error("No objects defined for arena: "+arenaConfig);
    for (auto& obj : temp_obj) {
        arena.objects[obj.first] = obj.second;
    }
  } catch(const ros::InvalidNameException& e) {
    ROS_ERROR_STREAM_NAMED("parser", "Invalid Name for arena config specified: " << arenaConfig << ": " << e.what());
    throw;
  }
  ROS_ASSERT(arena.workstations.size() > 1);

  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] Read " << arena.workstations.size() << " workstations from parameter server");


  std::string ppc_param = arenaConfig + "/cavities";
  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] try to read PP cavities from '" << ppc_param << "'");

  if( !nh.getParam(ppc_param, arena.cavities) )
    ROS_WARN_STREAM("No PPT cavities defined for Arena: " << arenaConfig);

  ROS_INFO_STREAM_NAMED("parser", "[REFBOX] read " << arena.cavities.size() << " PP cavities from parameter server");
  ROS_DEBUG_STREAM_NAMED("parser", "[REFBOX] Read Arena Definition:" << std::endl << arena);
  return arena;
}

static void filterObjects(TaskDefinitions& tasks, const ArenaDescription& arena) {
  for ( auto& item : tasks ) {
    auto& objects = item.second.objects;
    for ( const auto& obj : arena.objects ) {
      auto it = objects.find(obj.first);
      if (it == objects.end())
        objects.insert(obj);
    }
  }
}


void DefaultConfigParser::update() {
  auto arenaDescription = readArenaDefinition(arenaConfig());
  auto taskList = readTaskList(taskConfig());
  filterObjects(taskList, arenaDescription);
  mArena = arenaDescription;
  mTasks = taskList;
}

}
}

#include <atwork_commander_msgs/RefboxState.h>
#include <atwork_commander_msgs/StateUpdate.h>
#include <atwork_commander_msgs/StartTask.h>
#include <atwork_commander_msgs/GenerateTask.h>
#include <atwork_commander_msgs/LoadTask.h>

#include <ros/ros.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <regex>
#include <fstream>
#include <chrono>
#include <thread>

using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

enum class Command {
  NOTHING,
  FORWARD,
  STOP,
  START,
  GENERATE,
  STATE,
  STORE,
  LOAD
} command;

using atwork_commander_msgs::RefboxState;
using atwork_commander_msgs::StateUpdate;
using atwork_commander_msgs::StartTask;
using atwork_commander_msgs::GenerateTask;
using atwork_commander_msgs::LoadTask;
using atwork_commander_msgs::RobotHeader;
using atwork_commander_msgs::Task;

RefboxState state;
ros::Subscriber stateSub;
string refboxName = "atwork_commander";
vector<string> arguments;
bool verbose = false;
bool continous = false;
int result = 0;

static ostream& operator<<(ostream& os, Command cmd) {
  switch( cmd ) {
    case( Command::NOTHING ): return os << "nothing";
    case( Command::FORWARD ): return os << "\"forward state\"";
    case( Command::STOP    ): return os << "\"stop task\"";
    case( Command::START   ): return os << "\"start task\"";
    case( Command::GENERATE): return os << "\"generate task\"";
    case( Command::STATE   ): return os << "\"print refbox state\"";
    case( Command::STORE   ): return os << "\"store current task\"";
    case( Command::LOAD    ): return os << "\"load current task\"";
    default                 : return os << "UNKNOWN";
  };
}

static ostream& operator<<(ostream& os, const vector<string>& v) {
  os << "[";
  for( const auto& s: v )
    os << s << " ";
  return os << "]";
}

static bool forward() {
  size_t nextState;
  switch( state.state ) {
    case( RefboxState::PREPARATION ): nextState = RefboxState::EXECUTION; break;
    case( RefboxState::EXECUTION )  : nextState = RefboxState::READY; break;
    default:
      ROS_ERROR_STREAM_NAMED("forward", "[REFBOX-CONTROL] Cannot issue forward command: Refbox not in state PREPARATION or EXECUTION!");
      return false;
  }
  StateUpdate update;
  update.request.state = nextState;
  if ( !ros::service::call(refboxName+"/internal/state_update", update) ) {
    ROS_ERROR_STREAM_NAMED("forward", "[REFBOX-CONTROL] Issuing forward command failed: \n\t" << update.response.error);
    return false;
  }
  if( update.response.error.empty() )
    ROS_INFO_STREAM_NAMED("forward", "[REFBOX-CONTROL] Forwarding state of refbox successfull!");
  else {
    ROS_ERROR_STREAM_NAMED("forward", "[REFBOX-CONTROL] Forwarding state failed: " << update.response.error);
    return false;
  }
  return true;
}

static bool generate() {
  GenerateTask genTask;
  if( arguments.size() != 1 ) {
    ROS_ERROR_STREAM_NAMED("generate", "[REFBOX-CONTROL] Invalid amount of task types supplied: Expected 1: Got " << arguments.size());
    return true;
  }
  genTask.request.task_name = arguments[0];
  if ( !ros::service::call(refboxName+"/internal/generate_task", genTask) ) {
    ROS_ERROR_STREAM_NAMED("generate", "[REFBOX-CONTROL] Issuing generate command failed: \n\t" << genTask.response.error);
    return false;
  }
  if ( !genTask.response.error.empty() ) {
    ROS_ERROR_STREAM_NAMED("generate", "[REFBOX-CONTROL] Task generation failed");
    return false;
  } else
    ROS_INFO_STREAM_NAMED("generate", "[REFBOX-CONTROL] Generated Task:\n" << genTask.response.task);
  return true;
}

static bool stop() {
  StateUpdate update;
  update.request.state = RefboxState::READY;
  if ( !ros::service::call(refboxName+"/internal/state_update", update) ) {
    ROS_ERROR_STREAM_NAMED("stop", "[REFBOX-CONTROL] Issuing stop command failed");
    return false;
  }
  if ( !update.response.error.empty() ) {
    ROS_ERROR_STREAM_NAMED("stop", "[REFBOX-CONTROL] Stopping failed: \n\t" << update.response.error);
    return false;
  } else
    ROS_INFO_STREAM_NAMED("stop", "[REFBOX-CONTROL] Stopping task finished");
  return true;
}

static bool start() {
  StartTask startTask;

  startTask.request.robots.resize( arguments.size() );
  auto it = startTask.request.robots.begin();
  for( const string& name: arguments) {
    match_results<string::const_iterator> res;
    if( !regex_match(name, res, regex("([^[:space:]]+)/([^[:space:]]+)"))  || res.size() != 3) {
      it++;
      ROS_ERROR_STREAM_NAMED("start", "Invalid robot name encountered: " << name << "! Ignoring!");
      continue;
    }
    it->team_name = res[1];
    it->robot_name = res[2];
  }

  if( verbose ) {
    ostringstream os;
    for( const auto& r: startTask.request.robots)
      os << r << endl;
    ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Starting task on robots:" << os.str() );
  }

  if ( !ros::service::call(refboxName+"/internal/start_task", startTask) ) {
    ROS_ERROR_STREAM_NAMED("start", "[REFBOX-CONTROL] Issuing start command failed!");
    result = -1;
    return false;
  }

  if ( !startTask.response.error.empty() ) {
    ROS_ERROR_STREAM_NAMED("start", "[REFBOX-CONTROL] Starting task failed: \n\t" << startTask.response.error);
    result = -1;
    return false;
  } else {
    if ( arguments.empty() )
      ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on all registered robots!");
    else
      ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on following robots: " << arguments);
  }

  return true;
}

static void store() {
  if( arguments.size() != 1) {
    ROS_ERROR_STREAM_NAMED("store", "[REFBOX-CONTROL] wrong number of arguments supplied: required <fileName>");
    result = -1;
  }
  fs::path fileName = arguments[0];
  if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
  if( fs::exists(fileName) && !fs::is_regular_file(fileName) ) {
    ROS_ERROR_STREAM_NAMED("store", "[REFBOX-CONTROL] supplied file does exist and is not a regular file: " <<  fileName);
    result = -1;
  }

  if( fs::exists(fileName) && fs::is_regular_file(fileName) )
    ROS_WARN_STREAM_NAMED("store", "[REFBOX-CONTROL] supplied file does exist and will be overwritten: " <<  fileName);
  ROS_DEBUG_STREAM_NAMED("store", "[REFBOX-CONTROL] starting storage of current refbox task to " << arguments[0]);
  size_t size = ros::serialization::serializationLength(state.task);
  vector<uint8_t> buffer(size);
  ros::serialization::OStream stream(buffer.data(), size);
  ros::serialization::serialize(stream, state.task);
  ofstream file(arguments[0]);
  file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  file.close();
  ROS_INFO_STREAM_NAMED("store", "[REFBOX-CONTROL] current refbox task saved to " << fileName << " size: " << buffer.size());
}

static void load() {
  if( arguments.size() != 1) {
    ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] wrong number of arguments supplied: required <fileName>");
    result = -1;
  }
  fs::path fileName = arguments[0];
  if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
  if( !fs::exists(fileName) || !fs::is_regular_file(fileName) ) {
    ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] supplied file does not exits or is not a regular file: " <<  fileName);
    result = -1;
  }

  ROS_DEBUG_STREAM_NAMED("load", "[REFBOX-CONTROL] starting loading of task " << fileName << " to refbox");
  ifstream file(arguments[0]);
  vector<uint8_t> buffer;
  while( !file.eof()) {
    buffer.resize(buffer.size()+1024);
    file.read(reinterpret_cast<char*>(buffer.data())+buffer.size()-1024, 1024);
    ssize_t read=file.gcount();
    buffer.resize(buffer.size()-(1024-read));
  }
  ROS_DEBUG_STREAM_NAMED("load", "[REFBOX-CONTROL] Read a task of size " << buffer.size());
  ros::serialization::IStream stream(buffer.data(), buffer.size());
  LoadTask loadTask;
  ros::serialization::Serializer<Task>::read(stream, loadTask.request.task);
  if ( !ros::service::call(refboxName+"/internal/load_task", loadTask) ) {
    ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] Loading task into refbox failed!");
    result = -1;
  }
  if ( !loadTask.response.error.empty() ) {
    ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] Loading task failed: \n\t" << loadTask.response.error);
    result = -1;
  } else
    ROS_INFO_STREAM_NAMED("load", "[REFBOX-CONTROL] Loading task successfull: " << endl << loadTask.request.task);
}

static void printState(bool output) {
  ROS_INFO_STREAM_COND_NAMED(output, "state", "[REFBOX-CONTROL] Current refbox state: \n\t" << state);
}

static void stateUpdate(const RefboxState::ConstPtr msgPtr) {
  printState(verbose);
  state = *msgPtr;
  bool stopped = true;
  switch( command ) {
    case( Command::FORWARD  ): stopped = !continous || forward();  break;
    case( Command::STOP     ): stopped = !continous || stop();     break;
    case( Command::START    ): stopped = !continous || start();    break;
    case( Command::GENERATE ): stopped = !continous || generate(); break;
    case( Command::STORE    ): stopped =  true;  store();    break;
    case( Command::LOAD     ): stopped =  true;  load();     break;
    default: printState(!verbose); stopped = false;
  };
  if( stopped ) {
    if( result == 0)
      ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Command executed successfull! Shutting down!");
    else
      ROS_ERROR_STREAM_NAMED("control", "[REFBOX-CONTROL] Command failed! Shutting down!");
    ros::shutdown();
  }
}

static bool parseArgs(int argc, char** argv) {
  po::options_description desc("control [options] command [args]\nAllowed options");
  string cmdString;
  desc.add_options()
    ("command", po::value<string>(&cmdString), "command to execute [\"start <robots>\", \"generate <taskName>\", \"stop\", \"forward\", \"state\"]")
    ("args", po::value<vector<string>>(&arguments), "command arguments")
    ("verbose,v", po::value<bool>(&verbose), "set verbosity")
    ("continous,c", po::value<bool>(&continous), "try again after failure")
    ("help", "produce help message")
  ;
  po::positional_options_description p;
  p.add("command", 1).add("args", 2);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if( cmdString == "forward"  ) command = Command::FORWARD;
  if( cmdString == "stop"     ) command = Command::STOP;
  if( cmdString == "state"    ) command = Command::STATE;
  if( cmdString == "start"    ) command = Command::START;
  if( cmdString == "generate" ) command = Command::GENERATE;
  if( cmdString == "store"    ) command = Command::STORE;
  if( cmdString == "load"     ) command = Command::LOAD;

  if( command == Command::NOTHING || vm.count("help") ) {
    ROS_INFO_STREAM_NAMED("control", "[REFBOX-CONTROL] Syntax specification:\n" << desc);
    return false;
  }

  return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_helper");
    ros::NodeHandle nh;
    state.state = RefboxState::FAILURE;
    command = Command::NOTHING;

    if( !parseArgs(argc, argv) )
      return -1;

    if( verbose && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
      std::this_thread::sleep_for(std::chrono::seconds(1)); // Necessary to allow rosconsole to react to logger level change
    }

    if( !ros::param::get("~refbox", refboxName) )
      ROS_WARN_STREAM_NAMED("control", "[REFBOX-CONTROL] No Refbox name specified using \"" << refboxName << "\"!");

    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Command parsed successfull: Executing " << command << " with arguments " << arguments);

    stateSub = nh.subscribe(refboxName+"/internal/state", 1, &stateUpdate);

    while( ros::ok() )
      ros::spin();

    cout << "Shutdown finished" << endl;

    return result;
}

#include <atwork_commander/Control.hpp>

#include <ros/ros.h>

#include <boost/program_options.hpp>

#include <string>
#include <vector>
#include <regex>
#include <fstream>
#include <chrono>
#include <thread>
#include <memory>

using namespace std;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using atwork_commander::Control;

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

unique_ptr<Control> gControlPtr;
string refboxName = "atwork_commander";
vector<string> arguments;
bool verbose = false;
bool continous = false;
int result = 0;

static ostream& operator<<(ostream& os, const vector<string>& v) {
  os << "[";
  for( const auto& s: v )
    os << s << " ";
  return os << "]";
}

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

static bool forward() {
  return gControlPtr->forward();
}

static bool generate() {
  if( arguments.size() != 1 ) {
    ROS_ERROR_STREAM_NAMED("generate", "[REFBOX-CONTROL] Invalid amount of task types supplied: Expected 1: Got " << arguments.size());
    return true;
  }
  return gControlPtr->generate(arguments[0]);
}

static bool stop() {
  return gControlPtr->stop();
}

static bool start() {
  return gControlPtr->start(arguments);
}

static void store() {
  if( arguments.size() != 1) {
    ROS_ERROR_STREAM_NAMED("store", "[REFBOX-CONTROL] wrong number of arguments supplied: required <fileName>");
    result = -1;
  }
  fs::path fileName = arguments[0];
  if( !gControlPtr->store(fileName))
    result = -1;
}

static void load() {
  if( arguments.size() != 1) {
    ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] wrong number of arguments supplied: required <fileName>");
    result = -1;
  }
  fs::path fileName = arguments[0];
  if( !gControlPtr->load(fileName))
    result = -1;
}

static void stateUpdate(const Control::RefboxState& state) {
  ROS_INFO_STREAM_COND_NAMED(verbose, "state", "[REFBOX-CONTROL] Current refbox state: \n\t" << state);
  bool stopped = true;
  switch( command ) {
    case( Command::FORWARD  ): stopped = forward()  || !continous; break;
    case( Command::STOP     ): stopped = stop()     || !continous; break;
    case( Command::START    ): stopped = start()    || !continous; break;
    case( Command::GENERATE ): stopped = generate() || !continous; break;
    case( Command::STORE    ): stopped =  true;  store();    break;
    case( Command::LOAD     ): stopped =  true;  load();     break;
    default: stopped = false;
            ROS_INFO_STREAM_NAMED("state", "[REFBOX-CONTROL] Current refbox state: \n\t" << state);
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
    command = Command::NOTHING;

    if( !parseArgs(argc, argv) )
      return -1;

    if( verbose && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
      std::this_thread::sleep_for(std::chrono::seconds(1)); // Necessary to allow rosconsole to react to logger level change
    }

    gControlPtr.reset(new Control());

    if( ros::param::get("~refbox", refboxName) )
      gControlPtr->refbox(refboxName);
    else
      ROS_WARN_STREAM_NAMED("control", "[REFBOX-CONTROL] No Refbox name specified using \"" << gControlPtr->refbox() << "\"!");

    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Command parsed successfull: Executing " << command << " with arguments " << arguments);
    
    gControlPtr->stateUpdateCallback(&stateUpdate);

    while( ros::ok() )
      ros::spin();

    cout << "Shutdown finished" << endl;

    return result;
}

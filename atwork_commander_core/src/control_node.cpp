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
using atwork_commander::ControlError;

enum class Command {
  NOTHING,
  FORWARD,
  STOP,
  START,
  GENERATE,
  STATE,
  STORE,
  LOAD,
  TEST
} command;

enum class Result {
  OK,
  ERROR,
  INVALID_ARGUMENT,
  CONTINUE,
  NOTHING
};

unique_ptr<Control> gControlPtr;
string refboxName = "atwork_commander";
vector<string> arguments;
bool verbose = false;
bool continous = false;
unsigned int commandState = 0;

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
    case( Command::TEST    ): return os << "\"send task to robot for testing\"";
    default                 : return os << "UNKNOWN";
  };
}

static Result forward() {
  try {
    gControlPtr->forward();
    return Result::OK;
  }
  catch(const ControlError& e) {
    ROS_ERROR_STREAM("[REFBOX-CONTROL] Error in forwarding state occured! " << e.what());
    return Result::ERROR;
  }
}

static Result generate() {
  if( arguments.size() != 1 ) {
    ROS_ERROR_STREAM_NAMED("generate", "[REFBOX-CONTROL] Invalid amount of task types supplied: Expected 1: Got " << arguments.size());
    return Result::INVALID_ARGUMENT;
  }
  try {
    gControlPtr->generate(arguments[0]);
    return Result::OK;
  } catch(const ControlError& e) {
    ROS_ERROR_STREAM("[REFBOX-CONTROL] Error during task generation occured! " << e.what());
    return Result::ERROR;
  }
}

static Result stop() {
  try {
    gControlPtr->stop();
    return Result::OK;
  } catch(const ControlError& e) {
    ROS_ERROR_STREAM("[REFBOX-CONTROL] Error during stopping occured! " << e.what());
    return Result::ERROR;
  }
}

static Result start() {
  try {
    gControlPtr->start(arguments);
    return Result::OK;
  } catch(const ControlError& e) {
    ROS_ERROR_STREAM("[REFBOX-CONTROL] Error during starting task occured! " << e.what());
    return Result::ERROR;
  }
}

static Result store() {
  if( arguments.size() != 1) {
    ROS_ERROR_STREAM_NAMED("store", "[REFBOX-CONTROL] wrong number of arguments supplied: required <fileName>");
    return Result::INVALID_ARGUMENT;
  }
  fs::path fileName = arguments[0];
  try {
    gControlPtr->store(fileName);
    return Result::OK;
  }
  catch(const ControlError& e) {
    ROS_ERROR_STREAM("[REFBOX-CONTROL] Error in storing task occured! " << e.what());
    return Result::ERROR;
  }
}

static Result load() {
  if( arguments.size() != 1) {
    ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] wrong number of arguments supplied: required <fileName>");
    return Result::INVALID_ARGUMENT;
  }
  fs::path fileName = arguments[0];
  try {
    gControlPtr->load(fileName);
    return Result::OK;
  }
  catch(const ControlError& e) {
    ROS_ERROR_STREAM("[REFBOX-CONTROL] Error in loading task occured! " << e.what());
    return Result::ERROR;
  }
}

static Result test(unsigned int& commandState) {
  Result temp;
  switch(commandState) {
    case(0):
      ROS_DEBUG_STREAM("[REFBOX-CONTROL] Generate task for robot test.");
      temp = generate();
      if(temp == Result::OK) {
        ROS_DEBUG_STREAM("[REFBOX-CONTROL] generation successfull.");
        commandState = 1;
        arguments.clear();
        return Result::CONTINUE;
      }
      return temp;
    case(1):
      ROS_DEBUG_STREAM("[REFBOX-CONTROL] start task for robot test.");
      temp = start();
      if(temp == Result::OK) {
        ROS_DEBUG_STREAM("[REFBOX-CONTROL] start successfull.");
        commandState = 2;
        return Result::CONTINUE;
      }
      return temp;
    case(2):
      ROS_DEBUG_STREAM("[REFBOX-CONTROL] start execution of task for robot test.");
      return forward();
    }
    return Result::ERROR;
}

static void stateUpdate(const Control::RefboxState& state) {
  ROS_INFO_STREAM_COND_NAMED(verbose, "state", "[REFBOX-CONTROL] Current refbox state: \n\t" << state);
  Result res;
  switch( command ) {
    case( Command::FORWARD  ): res = forward();          break;
    case( Command::STOP     ): res = stop();             break;
    case( Command::START    ): res = start();            break;
    case( Command::GENERATE ): res = generate();         break;
    case( Command::TEST     ): res = test(commandState); break;
    case( Command::STORE    ): res = store();            break;
    case( Command::LOAD     ): res = load();             break;
    default: res = Result::NOTHING;
            ROS_INFO_STREAM_NAMED("state", "[REFBOX-CONTROL] Current refbox state: \n\t" << state);
  };

  if( res == Result::OK || res == Result::CONTINUE)
    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Command executed successfull!");
  else
    ROS_ERROR_STREAM_NAMED("control", "[REFBOX-CONTROL] Command failed!");
  if(continous && (res == Result::OK || res == Result::INVALID_ARGUMENT)) {
    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Shutting down!");
    ros::shutdown();
  }
}

static bool parseArgs(int argc, char** argv) {
  po::options_description desc("control [options] command [args]\nAllowed options");
  string cmdString;
  desc.add_options()
    ("command", po::value<string>(&cmdString), "command to execute [\"start <robots>\", \"generate <taskName>\", \"stop\", \"forward\", \"state\", \"test <taskName>\"]")
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
  if( cmdString == "test"     ) {command = Command::TEST; continous = true;}

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

    gControlPtr.reset(new Control());

    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Command parsed successfull: Executing " << command << " with arguments " << arguments);
    
    gControlPtr->stateUpdateCallback(&stateUpdate);

    while( ros::ok() )
      ros::spin();

    cout << "Shutdown finished" << endl;

    return 0;
}

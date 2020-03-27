#include <atwork_commander_msgs/RefboxState.h>
#include <atwork_commander_msgs/StateUpdate.h>
#include <atwork_commander_msgs/StartTask.h>
#include <atwork_commander_msgs/GenerateTask.h>

#include <ros/ros.h>

#include <boost/program_options.hpp>

#include <string>
#include <vector>
#include <regex>

using namespace std;

namespace po = boost::program_options;

enum class Command {
  NOTHING,
  FORWARD,
  STOP,
  START,
  GENERATE,
  STATE
} command;

using atwork_commander_msgs::RefboxState;
using atwork_commander_msgs::StateUpdate;
using atwork_commander_msgs::StartTask;
using atwork_commander_msgs::GenerateTask;
using atwork_commander_msgs::RobotHeader;

RefboxState state;
ros::Subscriber stateSub;
string refboxName = "default";
vector<string> arguments;
bool verbose = false;

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
  if ( !genTask.response.error.empty() )
    ROS_ERROR_STREAM_NAMED("generate", "[REFBOX-CONTROL] Task generation failed");
  else
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
  if ( !update.response.error.empty() )
    ROS_ERROR_STREAM_NAMED("stop", "[REFBOX-CONTROL] Stopping failed: \n\t" << update.response.error);
  else
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
    return false;
  }
  if ( !startTask.response.error.empty() )
    ROS_ERROR_STREAM_NAMED("start", "[REFBOX-CONTROL] Starting task failed: \n\t" << startTask.response.error);
  else {
    if ( arguments.empty() )
      ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on all registered robots!");
    else {
      ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on following robots: " << arguments);
    }
  }
  return true;
}

static void printState(bool output) {
  ROS_INFO_STREAM_COND_NAMED(output, "state", "[REFBOX-CONTROL] Current refbox state: \n\t" << state);
}

static void stateUpdate(const RefboxState::ConstPtr msgPtr) {
  printState(verbose);
  state = *msgPtr;
  bool result = true;
  switch( command ) {
    case( Command::FORWARD ): result = forward(); break;
    case( Command::STOP ): result = stop(); break;
    case( Command::START ): result = start(); break;
    case( Command::GENERATE ): result = generate(); break;
    default: printState(!verbose); result = false;
  };
  if( result )
    ros::shutdown();
}

bool parseArgs(int argc, char** argv) {
  po::options_description desc("control [options] command [args]\nAllowed options");
  string cmdString;
  desc.add_options()
    ("command", po::value<string>(&cmdString), "command to execute [\"start <robots>\", \"generate <taskName>\", \"stop\", \"forward\", \"state\"]")
    ("args", po::value<vector<string>>(&arguments), "command arguments")
    ("verbose", "increase verbosity")
    ("help", "produce help message")
  ;
  po::positional_options_description p;
  p.add("command", 1).add("args", 2);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);


  if( vm.count("verbose") ) verbose = true;

  if( cmdString == "forward"  ) command = Command::FORWARD;
  if( cmdString == "stop"     ) command = Command::STOP;
  if( cmdString == "state"    ) command = Command::STATE;
  if( cmdString == "start"    ) command = Command::START;
  if( cmdString == "generate" ) command = Command::GENERATE;

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

    if( !ros::param::get("~refbox", refboxName) )
      ROS_WARN_STREAM_NAMED("control", "[REFBOX-CONTROL] No Refbox name specified using \"default\"!");

    stateSub = nh.subscribe(refboxName+"/internal/state", 1, &stateUpdate);

    while( ros::ok() )
      ros::spin();

    return 0;
}

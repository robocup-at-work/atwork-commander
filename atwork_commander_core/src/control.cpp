#include "atwork_commander/Control.hpp"

#include <atwork_commander_msgs/StateUpdate.h>
#include <atwork_commander_msgs/StartTask.h>
#include <atwork_commander_msgs/GenerateTask.h>
#include <atwork_commander_msgs/LoadTask.h>
#include <atwork_commander_msgs/StateName.h>
#include <atwork_commander_msgs/RobotHeader.h>

#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <iostream>
#include <sstream>
#include <regex>

using namespace std;
namespace fs = boost::filesystem;

using atwork_commander_msgs::RefboxState;
using atwork_commander_msgs::StateUpdate;
using atwork_commander_msgs::StartTask;
using atwork_commander_msgs::GenerateTask;
using atwork_commander_msgs::LoadTask;
using atwork_commander_msgs::RobotHeader;
using atwork_commander_msgs::Task;
using atwork_commander_msgs::stateName;

static ostream& operator<<(ostream& os, const vector<RobotHeader>& v) {
  os << "[";
  for( const RobotHeader& r: v )
    os << r.team_name << "/" << r.robot_name << " ";
  return os << "]";
}

static ostream& operator<<(ostream& os, const vector<string>& v) {
  os << "[";
  for( const auto& s: v )
    os << s << " ";
  return os << "]";
}

namespace atwork_commander {

using Callback = Control::StateUpdateCallback;

static std::string prefix = "[ATC-CTRL] ";

template<typename T>
void call(string cmd, string service, T& arg) {
 if ( !ros::service::call(service, arg) )
    throw ControlError(cmd, ControlError::Reasons::CONNECTION_ERROR, service);

  if( !arg.response.error.empty() )
   throw ControlError(cmd, ControlError::Reasons::SERVICE_ERROR, arg.response.error);
}

static string genErrorMsg(string cmd, ControlError::Reasons reason, string argument) {
  ostringstream os;
  os << "Command " << cmd << ": ";
  switch(reason) {
    case(ControlError::Reasons::PATH_INVALID):     os << prefix << argument << " is not a valid path";                    break;
    case(ControlError::Reasons::TASK_INVALID):     os << prefix << argument << " is not a valid task name";               break;
    case(ControlError::Reasons::STATE_INVALID):    os << prefix << "Invalid for state " << argument;                      break;
    case(ControlError::Reasons::NO_TASK):          os << prefix << "No task registered";                                  break;
    case(ControlError::Reasons::NO_ROBOT):         os << prefix << "No robot registered";                                 break;
    case(ControlError::Reasons::SERVICE_ERROR):    os << prefix << "Service call to \"" << argument << "\" failed";       break;
    case(ControlError::Reasons::CONNECTION_ERROR): os << prefix << "Connection to service \"" << argument << "\" failed"; break;
    case(ControlError::Reasons::ARGUMENT_INVALID): os << prefix << "Argument of service " << argument << " invalid";      break;
    default:                                       os << prefix << "Unknown error";
  }
  return os.str();
}

ControlError::ControlError(string cmd, ControlError::Reasons reason, string argument)
  : runtime_error(genErrorMsg(cmd, reason, argument)), mReason(reason), mArgument(argument), mCommand(cmd)
  {}

class ControlImpl {
public:
  RefboxState state;
  Callback callback;
  string refbox = "atwork_commander";
  bool verbose = false;

  ros::Subscriber stateSub;

  void stateUpdate( RefboxState::ConstPtr msgPtr ) {
    state = *msgPtr;
    if( callback )
      callback(state);
  }

  ControlImpl() {
    
    if( !ros::param::get("~refbox", refbox) )
      ROS_WARN_STREAM_NAMED("control", prefix << "No Refbox name specified using \"" << refbox << "\"!");

    state.state = RefboxState::FAILURE;
    stateSub = ros::NodeHandle().subscribe(refbox+"/internal/state", 1, &ControlImpl::stateUpdate, this);
  }

  void forward() {
    size_t nextState;

    switch( state.state ) {
      case( RefboxState::PREPARATION ): nextState = RefboxState::EXECUTION; break;
      case( RefboxState::EXECUTION )  : nextState = RefboxState::READY; break;
      default:
        throw ControlError("forward", ControlError::Reasons::STATE_INVALID, stateName(state.state));
    }

    StateUpdate update;
    update.request.state = nextState;

    call("forward", refbox+"/internal/state_update", update);

    ROS_DEBUG_STREAM_NAMED("control", prefix << "Forwarding " << stateName(state.state) << " -> " << stateName(nextState) << " successfull!");
  }

  Task generate(const string& task) {
    GenerateTask genTask;
    genTask.request.task_name = task;

    call("generate", refbox+"/internal/generate_task", genTask);

    ROS_DEBUG_STREAM_NAMED("control", prefix << "Generate task \"" << task << "\" successfull: " << genTask.response.task);

    return genTask.response.task;
  }

  void stop() {
    StateUpdate update;
    switch( state.state ) {
      case( RefboxState::READY ):
      case( RefboxState::PREPARATION ):
      case( RefboxState::EXECUTION )  : update.request.state = RefboxState::READY; break;
      case( RefboxState::IDLE)        : update.request.state = RefboxState::IDLE; break;
      default:
        throw ControlError("stop", ControlError::Reasons::STATE_INVALID, stateName(state.state));
    }

    call("stop", refbox+"/internal/state_update", update);

    ROS_DEBUG_STREAM_NAMED("control", prefix << "stopping task successfull!");
  }

  void start(vector<string> robots={}) {
    StartTask startTask;
    if( state.state != RefboxState::READY)
      throw ControlError("start", ControlError::Reasons::STATE_INVALID, stateName(state.state));
    startTask.request.robots.resize( robots.size() );
    auto it = startTask.request.robots.begin();
    for( const string& name: robots) {
      match_results<string::const_iterator> res;
      if( !regex_match(name, res, regex("([^[:space:]]+)/([^[:space:]]+)"))  || res.size() != 3)
        throw ControlError("start", ControlError::Reasons::ARGUMENT_INVALID, name);
      it->team_name = res[1];
      it->robot_name = res[2];
      it++;
    }

    ROS_DEBUG_STREAM_NAMED("start", prefix << "Starting task on robots:" << startTask.request.robots );

    call("start", refbox+"/internal/start_task", startTask);


    if ( robots.empty() )
      ROS_DEBUG_STREAM_NAMED("start", prefix << "Started task on all registered robots!");
    else
      ROS_DEBUG_STREAM_NAMED("start", prefix << "Started task on following robots: " << robots);
  }

  void store(fs::path fileName) {
    if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
    if( fs::exists(fileName) && !fs::is_regular_file(fileName) )
      throw ControlError("store", ControlError::Reasons::PATH_INVALID, fileName.native());

    if( fs::exists(fileName) && fs::is_regular_file(fileName) )
      ROS_WARN_STREAM_NAMED("control", prefix << "supplied file does exist and will be overwritten: " <<  fileName);

    ROS_DEBUG_STREAM_NAMED("control", prefix << "starting storage of current refbox task to " << fileName << ":\n" << state.task);
    size_t size = ros::serialization::serializationLength(state.task);
    vector<uint8_t> buffer(size);
    ros::serialization::OStream stream(buffer.data(), size);
    ros::serialization::serialize(stream, state.task);
    fs::ofstream file(fileName);
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    file.close();
    ROS_DEBUG_STREAM_NAMED("store", prefix << "current refbox task saved to " << fileName << " size: " << buffer.size());
  }

  void load(fs::path fileName) {
    if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
    if( !fs::exists(fileName) || !fs::is_regular_file(fileName) )
      throw ControlError("load", ControlError::Reasons::PATH_INVALID, fileName.native());

    ROS_DEBUG_STREAM_NAMED("control", prefix << "starting loading of task from " << fileName << " to refbox");
    fs::ifstream file(fileName);
    vector<uint8_t> buffer;
    while( !file.eof()) {
      buffer.resize(buffer.size()+1024);
      file.read(reinterpret_cast<char*>(buffer.data())+buffer.size()-1024, 1024);
      ssize_t read=file.gcount();
      buffer.resize(buffer.size()-(1024-read));
    }
    ROS_DEBUG_STREAM_NAMED("control", prefix << "read a task of size " << buffer.size());
    ros::serialization::IStream stream(buffer.data(), buffer.size());
    LoadTask loadTask;
    ros::serialization::Serializer<Task>::read(stream, loadTask.request.task);
    call("load", refbox+"/internal/load_task", loadTask);
  }
};

Control::Control()
  : mImpl(new ControlImpl())
{}

Control::~Control() noexcept { delete mImpl; }

// Functions
Task Control::generate(const string& task)              { return mImpl->generate(task);  }
void Control::start( Robots robots )                    { mImpl->start(robots);   }
void Control::forward()                                 { mImpl->forward();       }
void Control::stop()                                    { mImpl->stop();          }
void Control::store( boost::filesystem::path fileName ) { mImpl->store(fileName); }
void Control::load( boost::filesystem::path fileName )  { mImpl->load(fileName);  }

//Getter
const Control::RefboxState& Control::state() const               { return mImpl->state;    }
const Callback&             Control::stateUpdateCallback() const { return mImpl->callback; }
const std::string&          Control::refbox() const              { return mImpl->refbox;   }

//Setter
void Control::refbox(const std::string& name)           { mImpl->refbox = name;       }
void Control::stateUpdateCallback( Callback callback )  { mImpl->callback = callback; };
}


ostream& operator<<(ostream& os, atwork_commander::ControlError::Reasons r) {
  switch(r) {
    case(atwork_commander::ControlError::Reasons::PATH_INVALID     ): return os << "PATH_INVALID"; break;
    case(atwork_commander::ControlError::Reasons::TASK_INVALID     ): return os << "TASK_INVALID"; break;
    case(atwork_commander::ControlError::Reasons::STATE_INVALID    ): return os << "STATE_INVALID"; break;
    case(atwork_commander::ControlError::Reasons::NO_TASK          ): return os << "NO_TASK"; break;
    case(atwork_commander::ControlError::Reasons::NO_ROBOT         ): return os << "NO_ROBOT"; break;
    case(atwork_commander::ControlError::Reasons::SERVICE_ERROR    ): return os << "SERVICE_ERROR"; break;
    case(atwork_commander::ControlError::Reasons::CONNECTION_ERROR ): return os << "CONNECTION_ERROR"; break;
    default                                                         : return os << "UNKNOWN";
  };
}

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

template<typename T>
void call(std::string service, T& arg) {
 if ( !ros::service::call(service, arg) )
    throw ControlError(ControlError::Reasons::CONNECTION_ERROR, service);

  if( !arg.response.error.empty() )
   throw ControlError(ControlError::Reasons::SERVICE_ERROR, arg.response.error);
}

static string genErrorMsg(ControlError::Reasons reason, std::string argument) {
  ostringstream os;
  switch(reason) {
    case(ControlError::Reasons::PATH_INVALID): os << argument << " is not a valid path"; break;
    case(ControlError::Reasons::TASK_INVALID): os << argument << " is not a valid task name"; break;
    case(ControlError::Reasons::STATE_INVALID): os << "Command invalid for state " << argument; break;
    case(ControlError::Reasons::NO_TASK): os << "No task registered"; break;
    case(ControlError::Reasons::NO_ROBOT): os << "No robot registered"; break;
    case(ControlError::Reasons::SERVICE_ERROR): os << "Service call to \"" << argument << "\" failed"; break;
    case(ControlError::Reasons::CONNECTION_ERROR): os << "Connection to service \"" << argument << "\" failed"; break;
    default: os << "Unknown error";
  }
  return os.str();
}

ControlError::ControlError(ControlError::Reasons reason, string argument)
  : runtime_error(genErrorMsg(reason, argument)), mReason(reason), mArgument(argument)
  {}

class ControlImpl {
public:
  RefboxState state;
  Callback callback;
  string refbox;
  bool verbose = false;

  ros::Subscriber stateSub;

  void stateUpdate( RefboxState::ConstPtr msgPtr ) {
    state = *msgPtr;
    if( callback )
      callback(state);
  }

  ControlImpl( string name )
    : refbox(name)
  {

    state.state = RefboxState::FAILURE;
    stateSub = ros::NodeHandle().subscribe(refbox+"/internal/state", 1, &ControlImpl::stateUpdate, this);
  }

  void forward() {
    size_t nextState;

    switch( state.state ) {
      case( RefboxState::PREPARATION ): nextState = RefboxState::EXECUTION; break;
      case( RefboxState::EXECUTION )  : nextState = RefboxState::READY; break;
      default:
        throw ControlError(ControlError::Reasons::STATE_INVALID, stateName(state.state));
    }

    StateUpdate update;
    update.request.state = nextState;

    call(refbox+"/internal/state_update", update);

    ROS_DEBUG_STREAM_NAMED("control", "[CONTROL] Forwarding " << stateName(state.state) << " -> " << stateName(nextState) << " successfull!");
  }

  Task generate(const string& task) {
    GenerateTask genTask;
    genTask.request.task_name = task;

    call(refbox+"/internal/generate_task", genTask);

    ROS_DEBUG_STREAM_NAMED("control", "[CONTROL] Generate task \"" << task << "\" successfull: " << genTask.response.task);

    return genTask.response.task;
  }

  void stop() {
    StateUpdate update;
    update.request.state = RefboxState::READY;
    call(refbox+"/internal/state_update", update);
    ROS_DEBUG_STREAM_NAMED("control", "[CONTROL] stopping task successfull!");
  }

  void start(vector<string> robots={}) {
    StartTask startTask;

    startTask.request.robots.resize( robots.size() );
    auto it = startTask.request.robots.begin();
    for( const string& name: robots) {
      match_results<string::const_iterator> res;
      if( !regex_match(name, res, regex("([^[:space:]]+)/([^[:space:]]+)"))  || res.size() != 3) {
        it++;
        ROS_ERROR_STREAM_NAMED("start", "Invalid robot name encountered: " << name << "! Ignoring!");
        continue;
      }
      it->team_name = res[1];
      it->robot_name = res[2];
    }

    ROS_DEBUG_STREAM_NAMED("start", "[REFBOX-CONTROL] Starting task on robots:" << startTask.request.robots );

    call(refbox+"/internal/start_task", startTask);

    if ( robots.empty() )
      ROS_DEBUG_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on all registered robots!");
    else
      ROS_DEBUG_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on following robots: " << robots);
  }

  void store(fs::path fileName) {
    if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
    if( fs::exists(fileName) && !fs::is_regular_file(fileName) )
      throw ControlError(ControlError::Reasons::PATH_INVALID, fileName.native());

    if( fs::exists(fileName) && fs::is_regular_file(fileName) )
      ROS_WARN_STREAM_NAMED("control", "[REFBOX-CONTROL] supplied file does exist and will be overwritten: " <<  fileName);

    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] starting storage of current refbox task to " << fileName << ":\n" << state.task);
    size_t size = ros::serialization::serializationLength(state.task);
    vector<uint8_t> buffer(size);
    ros::serialization::OStream stream(buffer.data(), size);
    ros::serialization::serialize(stream, state.task);
    fs::ofstream file(fileName);
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    file.close();
    ROS_DEBUG_STREAM_NAMED("store", "[REFBOX-CONTROL] current refbox task saved to " << fileName << " size: " << buffer.size());
  }

  void load(fs::path fileName) {
    if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
    if( !fs::exists(fileName) || !fs::is_regular_file(fileName) )
      throw ControlError(ControlError::Reasons::PATH_INVALID, fileName.native());

    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] starting loading of task from " << fileName << " to refbox");
    fs::ifstream file(fileName);
    vector<uint8_t> buffer;
    while( !file.eof()) {
      buffer.resize(buffer.size()+1024);
      file.read(reinterpret_cast<char*>(buffer.data())+buffer.size()-1024, 1024);
      ssize_t read=file.gcount();
      buffer.resize(buffer.size()-(1024-read));
    }
    ROS_DEBUG_STREAM_NAMED("control", "[REFBOX-CONTROL] Read a task of size " << buffer.size());
    ros::serialization::IStream stream(buffer.data(), buffer.size());
    LoadTask loadTask;
    ros::serialization::Serializer<Task>::read(stream, loadTask.request.task);
    call(refbox+"/internal/load_task", loadTask);
  }
};

Control::Control(string name)
  : mImpl(new ControlImpl(name))
{}

Control::~Control() { delete mImpl; }

// Functions
Task Control::generate(const string& task)              { return mImpl->generate(task);  }
void Control::start( Robots robots )                    { mImpl->start(robots);   }
void Control::forward()                                 { mImpl->forward();       }
void Control::stop()                                    { mImpl->stop();          }
void Control::store( boost::filesystem::path fileName ) { mImpl->store(fileName); }
void Control::load( boost::filesystem::path fileName )  { mImpl->load(fileName);  }

//Getter
bool                        Control::verbose() const             { return mImpl->verbose;  }
const Control::RefboxState& Control::state() const               { return mImpl->state;    }
const Callback&             Control::stateUpdateCallback() const { return mImpl->callback; }
const std::string&          Control::refbox() const              { return mImpl->refbox;   }

//Setter
void Control::refbox(const std::string& name)           { mImpl->refbox = name;       }
void Control::stateUpdateCallback( Callback callback )  { mImpl->callback = callback; };
void Control::verbose( bool value )                     { mImpl->verbose = value;     }
}

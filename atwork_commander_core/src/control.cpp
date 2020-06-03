#include "atwork_commander/Control.hpp"

#include <atwork_commander_msgs/StateUpdate.h>
#include <atwork_commander_msgs/StartTask.h>
#include <atwork_commander_msgs/GenerateTask.h>
#include <atwork_commander_msgs/LoadTask.h>

#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <iostream>
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

static ostream& operator<<(ostream& os, const vector<string>& v) {
  os << "[";
  for( const auto& s: v )
    os << s << " ";
  return os << "]";
}

namespace atwork_commander {

using Callback = Control::StateUpdateCallback;

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

  bool forward() {
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
    if ( !ros::service::call(refbox+"/internal/state_update", update) ) {
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

  bool generate(const string& task) {
    GenerateTask genTask;
    genTask.request.task_name = task;
    if ( !ros::service::call(refbox+"/internal/generate_task", genTask) ) {
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

  bool stop() {
    StateUpdate update;
    update.request.state = RefboxState::READY;
    if ( !ros::service::call(refbox+"/internal/state_update", update) ) {
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

  bool start(vector<string> robots={}) {
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

    if( verbose ) {
      ostringstream os;
      for( const auto& r: startTask.request.robots)
        os << r << endl;
      ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Starting task on robots:" << os.str() );
    }

    if ( !ros::service::call(refbox+"/internal/start_task", startTask) ) {
      ROS_ERROR_STREAM_NAMED("start", "[REFBOX-CONTROL] Issuing start command failed!");
      return false;
    }

    if ( !startTask.response.error.empty() ) {
      ROS_ERROR_STREAM_NAMED("start", "[REFBOX-CONTROL] Starting task failed: \n\t" << startTask.response.error);
      return false;
    } else {
      if ( robots.empty() )
        ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on all registered robots!");
      else
        ROS_INFO_STREAM_NAMED("start", "[REFBOX-CONTROL] Started task on following robots: " << robots);
    }

    return true;
  }

  bool store(fs::path fileName) {
    if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
    if( fs::exists(fileName) && !fs::is_regular_file(fileName) ) {
      ROS_ERROR_STREAM_NAMED("store", "[REFBOX-CONTROL] supplied file does exist and is not a regular file: " <<  fileName);
      return false;
    }

    if( fs::exists(fileName) && fs::is_regular_file(fileName) )
      ROS_WARN_STREAM_NAMED("store", "[REFBOX-CONTROL] supplied file does exist and will be overwritten: " <<  fileName);

    ROS_DEBUG_STREAM_NAMED("store", "[REFBOX-CONTROL] starting storage of current refbox task to " << fileName);
    size_t size = ros::serialization::serializationLength(state.task);
    vector<uint8_t> buffer(size);
    ros::serialization::OStream stream(buffer.data(), size);
    ros::serialization::serialize(stream, state.task);
    fs::ofstream file(fileName);
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    file.close();
    ROS_INFO_STREAM_NAMED("store", "[REFBOX-CONTROL] current refbox task saved to " << fileName << " size: " << buffer.size());
    return true;
  }

  bool load(fs::path fileName) {
    if( fileName.has_relative_path() ) fileName = fs::current_path() / fileName;
    if( !fs::exists(fileName) || !fs::is_regular_file(fileName) ) {
      ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] supplied file does not exits or is not a regular file: " <<  fileName);
      return false;
    }

    ROS_DEBUG_STREAM_NAMED("load", "[REFBOX-CONTROL] starting loading of task " << fileName << " to refbox");
    fs::ifstream file(fileName);
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
    if ( !ros::service::call(refbox+"/internal/load_task", loadTask) ) {
      ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] Loading task into refbox failed!");
      return false;
    }
    if ( !loadTask.response.error.empty() ) {
      ROS_ERROR_STREAM_NAMED("load", "[REFBOX-CONTROL] Loading task failed: \n\t" << loadTask.response.error);
      return false;
    } else
      ROS_INFO_STREAM_NAMED("load", "[REFBOX-CONTROL] Loading task successfull: " << endl << loadTask.request.task);
    return true;
  }
};

Control::Control(string name)
  : mImpl(new ControlImpl(name))
{}

Control::~Control() { delete mImpl; }

// Functions
bool Control::generate(const string& task)              { return mImpl->generate(task);  }
bool Control::start( Robots robots )                    { return mImpl->start(robots);   }
bool Control::forward()                                 { return mImpl->forward();       }
bool Control::stop()                                    { return mImpl->stop();          }
bool Control::store( boost::filesystem::path fileName ) { return mImpl->store(fileName); }
bool Control::load( boost::filesystem::path fileName )  { return mImpl->load(fileName);  }

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

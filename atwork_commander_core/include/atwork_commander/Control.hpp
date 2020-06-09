#pragma once

#include <atwork_commander_msgs/RefboxState.h>
#include <atwork_commander_msgs/Task.h>

#include <boost/filesystem/path.hpp>

#include <string>
#include <vector>

namespace atwork_commander {

class ControlImpl;

class ControlError : public std::runtime_error {
  public:
    enum class Reasons {
      PATH_INVALID,
      TASK_INVALID,
      STATE_INVALID,
      NO_TASK,
      NO_ROBOT,
      SERVICE_ERROR,
      CONNECTION_ERROR
    };
  private:
    Reasons mReason;
    std::string mArgument;
  public:
    ControlError(Reasons reason, std::string argument);
    Reasons reason() const { return mReason; }
    const std::string& argument() const { return mArgument; }
};

class Control {
    ControlImpl* mImpl;
  public:
    using Robots = std::vector< std::string >;
    using RefboxState = atwork_commander_msgs::RefboxState;
    using StateUpdateCallback = std::function<void(const RefboxState&)>;
    using Task = atwork_commander_msgs::Task;

    Control( std::string refboxName = "atwork_commander" );
    ~Control() noexcept;

    void verbose( bool value );
    bool verbose() const;
    const std::string& refbox() const;
    void refbox(const std::string& name);
    const RefboxState& state() const;
    void stateUpdateCallback(StateUpdateCallback callback);
    const StateUpdateCallback& stateUpdateCallback() const;

    Task generate(const std::string& task);
    void start( Robots robots = {} );
    void forward();
    void stop();
    void store( boost::filesystem::path fileName );
    void load( boost::filesystem::path fileName );
};

}

std::ostream& operator<<(std::ostream& os, atwork_commander::ControlError::Reasons r);

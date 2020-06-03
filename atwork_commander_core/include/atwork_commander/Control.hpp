#pragma once

#include <atwork_commander_msgs/RefboxState.h>

#include <boost/filesystem/path.hpp>

#include <string>
#include <vector>

namespace atwork_commander {

class ControlImpl;

class Control {
    ControlImpl* mImpl;
  public:
    using Robots = std::vector< std::string >;
    using RefboxState = atwork_commander_msgs::RefboxState;
    using StateUpdateCallback = std::function<void(const RefboxState&)>;

    Control( std::string refboxName = "atwork_commander" );
    ~Control();

    void verbose( bool value );
    bool verbose() const;
    const std::string& refbox() const;
    void refbox(const std::string& name);

    bool generate(const std::string& task);
    bool start( Robots robots = {} );
    bool forward();
    bool stop();
    bool store( boost::filesystem::path fileName );
    bool load( boost::filesystem::path fileName );
    const RefboxState& state() const;
    void stateUpdateCallback(StateUpdateCallback callback);
    const StateUpdateCallback& stateUpdateCallback() const;
};

}

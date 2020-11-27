#pragma once

#include <atwork_commander_gen/ConfigParserInterface.h>

namespace atwork_commander {
namespace task_generator {

class DefaultConfigParser : public ConfigParserInterface {
  protected:
    TaskDefinitions mTasks;
    ArenaDescription mArena;
    virtual void update();
  public:
    virtual const TaskDefinitions& tasks() const { return mTasks; }
    virtual const ArenaDescription& arena() const { return mArena; }
};

}
}

#pragma once

#include <atwork_commander_gen/ConfigParserInterface.h>



class DefaultConfigParser : public ConfigParserInterface {
  private:
    TaskDefinitions mTasks;
    ArenaDescription mArena;
  public:
    DefaultConfigParser(const std::string& taskConfig, const std::string& arenaConfig);
    virtual void reload(std::string taskConfig="", std::string arenaConfig="");
    const TaskDefinitions& tasks() const { return mTasks; }
    const ArenaDescription& arena() const { return mArena; }
};

}
}



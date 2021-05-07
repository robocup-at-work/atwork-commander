#pragma once

#include <atwork_commander_msgs/RefboxState.h>

namespace atwork_commander_msgs {

const char* stateName(unsigned int state) {
  switch(state) {
    case(RefboxState::FAILURE):     return "FAILURE";
    case(RefboxState::IDLE):        return "IDLE";
    case(RefboxState::READY):       return "READY";
    case(RefboxState::PREPARATION): return "PREPARATION";
    case(RefboxState::EXECUTION):   return "EXECUTION";
    default:                        return "UNKNOWN";
  }
}

}

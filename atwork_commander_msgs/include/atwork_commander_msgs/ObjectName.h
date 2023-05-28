#pragma once

#include <atwork_commander_msgs/Object.h>

namespace atwork_commander_msgs {

inline const char* objectName(uint16_t typeID) {

  switch(typeID) {
    case(Object::EMPTY):          return "Nothing";        break;
    case(Object::F20_20_B):       return "F20_20_B";       break;
    case(Object::F20_20_G):       return "F20_20_G";       break;
    case(Object::S40_40_B):       return "S40_40_B";       break;
    case(Object::S40_40_G):       return "S40_40_G";       break;
    case(Object::M20_100):        return "M20_100";        break;
    case(Object::M20):            return "M20";            break;
    case(Object::M30):            return "M30";            break;
    case(Object::R20):            return "R20";            break;
    case(Object::BEARING_BOX):    return "BEARING_BOX";    break;
    case(Object::BEARING):        return "BEARING";        break;
    case(Object::AXIS):           return "AXIS";           break;
    case(Object::DISTANCE_TUBE):  return "DISTANCE_TUBE";  break;
    case(Object::MOTOR):          return "MOTOR";          break;
    case(Object::Axis2):          return "Axis2";          break;
    case(Object::Bearing2):       return "Bearing2";       break;
    case(Object::Housing):        return "Housing";        break;
    case(Object::Motor2):         return "Motor2";         break;
    case(Object::Spacer):         return "Spacer";         break;
    case(Object::Screwdriver):    return "Screwdriver";    break;
    case(Object::Wrench):         return "Wrench";         break;
    case(Object::Drill):          return "Drill";          break;
    case(Object::AllenKey):       return "AllenKey";       break;
    case(Object::CONTAINER_RED):  return "CONTAINER_RED";  break;
    case(Object::CONTAINER_BLUE): return "CONTAINER_BLUE"; break;
    case(Object::F20_20_H):       return "F20_20_H";       break;
    case(Object::F20_20_V):       return "F20_20_V";       break;
    case(Object::F20_20_F):       return "F20_20_F";       break;
    case(Object::S40_40_H):       return "S40_40_H";       break;
    case(Object::S40_40_V):       return "S40_40_V";       break;
    case(Object::S40_40_F):       return "S40_40_F";       break;
    case(Object::M20_H):          return "M20_H";          break;
    case(Object::M20_V):          return "M20_V";          break;
    case(Object::M20_F):          return "M20_F";          break;
    case(Object::M20_100_H):      return "M20_100_H";      break;
    case(Object::M20_100_V):      return "M20_100_V";      break;
    case(Object::M20_100_F):      return "M20_100_F";      break;
    case(Object::M30_H):          return "M30_H";          break;
    case(Object::M30_V):          return "M30_V";          break;
    case(Object::M30_F):          return "M30_F";          break;
    case(Object::R20_H):          return "R20_H";          break;
    case(Object::R20_V):          return "R20_V";          break;
    case(Object::R20_F):          return "R20_F";          break;
    default:                      return "UNKNOWN";
  }
}

} // ns atwork_commander

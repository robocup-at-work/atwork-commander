#pragma once

#include <atwork_commander_msgs/Task.h>
#include <atwork_commander_msgs/Object.h>
#include <atwork_commander_msgs/Workstation.h>

#include <ros/console.h>

#include <unordered_map>

namespace atwork_commander {
namespace task_generator {
namespace jurek {

using namespace std;
struct Converter  {
  using Workstation = atwork_commander_msgs::Workstation;
  using Object = atwork_commander_msgs::Object;
  using Objects         = vector<Object>;
  using WorkstationObjs = unordered_map<string, Objects>;
  using Workstations    = vector<Workstation>;

  static bool compObjects(const Object& a, const Object& b) {
    return a.object<b.object || ( a.object==b.object && a.target<b.target );
  }

  static WorkstationObjs toMap(const Workstations& wsList) {
    WorkstationObjs objs(wsList.size());
      for( const Workstation& ws: wsList) {
        auto result = objs.emplace(ws.name, ws.objects);
        if( !result.second ) {
          ROS_ERROR_STREAM_THROTTLE_NAMED(60, "object", "[REFBOX-COM] Error in task! Workstation name exists multiple times: " << ws.name);
          continue;
        }
        sort(result.first->second.begin(), result.first->second.end(), compObjects);
      }
      return objs;
    }
    static WorkstationObjs intersect(const WorkstationObjs& a, WorkstationObjs& b) {
      WorkstationObjs intersection( a.size() );
      for(const auto& ws: a) {
        const auto& wsName = ws.first;
        const auto& aObjects = ws.second;
        const auto& bObjects = b[wsName];
        auto result = intersection.emplace(piecewise_construct, make_tuple(ws.first), make_tuple());
        set_intersection(aObjects.begin(), aObjects.end(),
                         bObjects.begin(), bObjects.end(),
                         back_insert_iterator<decltype(result.first->second)>(result.first->second),
                         compObjects
                        );
      }
      return intersection;
    }

    static WorkstationObjs diff(const WorkstationObjs& a, WorkstationObjs& b) {
      WorkstationObjs difference( a.size() );
      for(const auto& ws: a) {
        const auto& wsName = ws.first;
        const auto& aObjects = ws.second;
        const auto& bObjects = b[wsName];
        auto result = difference.emplace(piecewise_construct, make_tuple(ws.first), make_tuple());
        set_difference(aObjects.begin(), aObjects.end(),
                         bObjects.begin(), bObjects.end(),
                         back_insert_iterator<decltype(result.first->second)>(result.first->second),
                         compObjects
                        );
      }
      return difference;
    }

    static string findObject(WorkstationObjs& wsMap, const Object& o, const string& source, bool del=false) {
      for(auto& ws: wsMap) {
        if( ws.first == source ) continue;
        auto objIter = find_if(ws.second.begin(), ws.second.end(), [&o](const Object& b){return o.object == b.object && o.target == b.target;});
        if(objIter != ws.second.end()) {
          if(del) ws.second.erase(objIter);
          return ws.first;
         }
      }
      return "";
    }

    static int findContainer(const WorkstationObjs& wsMap, const Object& o, const string& source) {
      size_t i = 100;
      for(const auto& ws: wsMap)
        for(const Object& o2: ws.second) {
          if( o2.object == Object::CONTAINER_RED || o2.object == Object::CONTAINER_BLUE)
            i++;
          if( o.object == o2.object && ws.first == source)
            return i;
      }
      return -1;
    }
};

}
}
}

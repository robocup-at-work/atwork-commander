#include "atwork_commander_com/plugin_interface.h"
#include "atwork_commander_msgs/ObjectTask.h"
#include "atwork_commander_msgs/ObjectName.h"

#include <pluginlib/class_list_macros.h>

#include <unordered_set>
#include <unordered_map>
#include <set>
#include <vector>

using namespace std;
using atwork_commander_msgs::objectName;
using atwork_commander_msgs::Workstation;
using atwork_commander_msgs::Object;
using atwork_commander_msgs::Transport;
using atwork_commander_msgs::ObjectTask;
using Objects         = vector<Object>;
using WorkstationObjs = unordered_map<string, Objects>;
using Workstations    = vector<Workstation>;

static ostream& operator<<(ostream& os, const vector<Object>& objects) {
  os << "[";
  for(const Object& o: objects) {
    os << objectName(o.object);
    if( o.target != Object::EMPTY )
      os << "( " << objectName(o.target) <<" )";
    os << " ";
  }
  return os << "]";
}

static ostream& operator<<(ostream& os, const WorkstationObjs& wsMap) {
  for(const auto& ws: wsMap) {
    os << "\t" << ws.first << ": " << ws.second << endl;
  }
  return os;
}

namespace atwork_commander {
namespace com_plugin {
namespace ros {

class TaskObjectCentric : public Base {

    ::ros::NodeHandle rh;

    ::ros::Subscriber robot_state_sub;
    ::ros::Publisher send_task_pub;

     using WorkstationObjs = unordered_map<string, vector<Object>>;
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

    static const string& findObject(WorkstationObjs& wsMap, const Object& o, const string& source) {
      for(auto& ws: wsMap) {
        if( ws.first == source ) continue;
        auto it = find_if(ws.second.begin(), ws.second.end(), [&o](const Object& b){return o.object == b.object && o.target == b.target && !o.decoy;});
        if ( it != ws.second.end() ) {
            ws.second.erase(it);
            return ws.first;
        }
      }
      return source;
    }

public:
    virtual ~TaskObjectCentric() {}


    virtual void sendTask( atwork_commander_msgs::Task task ) {
      ObjectTask objects;
      objects.execute_on = task.execute_on;
      objects.prep_time = task.prep_time;
      objects.exec_time = task.exec_time;

      auto start  = toMap(task.arena_start_state);
      auto target = toMap(task.arena_target_state);

      ROS_DEBUG_STREAM_THROTTLE_NAMED(60, "object", "[REFBOX-COM] Start State of Arena: " << endl << start);
      ROS_DEBUG_STREAM_THROTTLE_NAMED(60, "object", "[REFBOX-COM] Target State of Arena Arena: " << endl << target);

      WorkstationObjs immobile = intersect(start, target);
      WorkstationObjs startObjs = diff(start, immobile);
      WorkstationObjs targetObjs = diff(target, immobile);

      ROS_DEBUG_STREAM_THROTTLE_NAMED(60, "object", "[REFBOX-COM] Immobile Objects in Arena: " << endl << immobile);
      ROS_DEBUG_STREAM_THROTTLE_NAMED(60, "object", "[REFBOX-COM] Start Objects: " << endl << startObjs);
      ROS_DEBUG_STREAM_THROTTLE_NAMED(60, "object", "[REFBOX-COM] Target Objects: " << endl << targetObjs);

      for(const auto& objs: startObjs)
        for( const Object& o: objs.second ) {
          Transport t;
          t.object = o;
          t.source = objs.first;
          t.destination = findObject(targetObjs, o, objs.first);
          objects.subtasks.push_back(t);
        }
      objects.id = task.id;
      objects.type = task.type;
      this->send_task_pub.publish( objects );
    }

    virtual void onInit( ::ros::NodeHandle roshandle ) {
        this->rh = roshandle;

        this->robot_state_sub = this->rh.subscribe( "robot_state", 10, &TaskObjectCentric::receiveRobotStateClb, this );
        this->send_task_pub = this->rh.advertise<atwork_commander_msgs::ObjectTask>("object_task", 1);
    }

private:
    void receiveRobotStateClb( const atwork_commander_msgs::RobotState::ConstPtr& msg ) {
      this->sendRobotState( *msg );
    }

};

}; // ns ros
}; // ns atwork_commander
}; // ns communication


PLUGINLIB_EXPORT_CLASS(atwork_commander::com_plugin::ros::TaskObjectCentric, atwork_commander::com_plugin::Base)

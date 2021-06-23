#include "atwork_commander_com/plugin_interface.h"

#include <pluginlib/class_list_macros.h>

#include <algorithm>

namespace atwork_commander {
namespace com_plugin {
namespace ros {

using namespace std;
using namespace atwork_commander_msgs;

class TaskArenaCentric : public Base {

    ::ros::NodeHandle rh;

    ::ros::Subscriber robot_state_sub;
    ::ros::Publisher send_task_pub;

public:
    TaskArenaCentric() {}
    virtual ~TaskArenaCentric() {}

    virtual void sendTask( atwork_commander_msgs::Task task ) {
      for(atwork_commander_msgs::Workstation& ws : task.arena_start_state) {
        auto endIt = remove_if(ws.objects.begin(), ws.objects.end(), [](const Object& o){ return o.decoy;});
        ws.objects.erase(endIt, ws.objects.end());
      }
      for(atwork_commander_msgs::Workstation& ws : task.arena_target_state) {
        auto endIt = remove_if(ws.objects.begin(), ws.objects.end(), [](const Object& o){ return o.decoy;});
        ws.objects.erase(endIt, ws.objects.end());
      }

        this->send_task_pub.publish( task );
    }

    virtual void onInit( ::ros::NodeHandle roshandle ) {
        this->rh = roshandle;

        this->robot_state_sub = this->rh.subscribe( "robot_state", 10, &TaskArenaCentric::receiveRobotStateClb, this );
        this->send_task_pub = this->rh.advertise<Task>("task", 1);
    }

private:
    void receiveRobotStateClb( const RobotState::ConstPtr& msg ) {
      this->sendRobotState( *msg );
    }

};

}; // ns ros
}; // ns com_plugin
}; // ns atwork_commander


PLUGINLIB_EXPORT_CLASS(atwork_commander::com_plugin::ros::TaskArenaCentric, atwork_commander::com_plugin::Base)

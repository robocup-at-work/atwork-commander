#include "atwork_commander_com/plugin_interface.h"

#include <pluginlib/class_list_macros.h>

namespace atwork_commander {
namespace com_plugin {
namespace ros {

static void cleanDecoys(std::vector<::atwork_commander_msgs::Workstation>& state) {
  for(atwork_commander_msgs::Workstation& ws: state){
    auto newEnd = remove_if(ws.objects.begin(), ws.objects.end(), [](const atwork_commander_msgs::Object& obj){return obj.decoy;});
    ws.objects.erase(newEnd, ws.objects.end());
  }
}

class TaskArenaCentric : public Base {

    ::ros::NodeHandle rh;

    ::ros::Subscriber robot_state_sub;
    ::ros::Publisher send_task_pub;

public:
    TaskArenaCentric() {}
    virtual ~TaskArenaCentric() {}

    virtual void sendTask( atwork_commander_msgs::Task task ) {
        cleanDecoys(task.arena_start_state);
        cleanDecoys(task.arena_target_state);
        this->send_task_pub.publish( task );
    }

    virtual void onInit( ::ros::NodeHandle roshandle ) {
        this->rh = roshandle;

        this->robot_state_sub = this->rh.subscribe( "robot_state", 10, &TaskArenaCentric::receiveRobotStateClb, this );
        this->send_task_pub = this->rh.advertise<atwork_commander_msgs::Task>("task", 1);
    }

private:
    void receiveRobotStateClb( const atwork_commander_msgs::RobotState::ConstPtr& msg ) {
      this->sendRobotState( *msg );
    }

};

}; // ns ros
}; // ns com_plugin
}; // ns atwork_commander


PLUGINLIB_EXPORT_CLASS(atwork_commander::com_plugin::ros::TaskArenaCentric, atwork_commander::com_plugin::Base)

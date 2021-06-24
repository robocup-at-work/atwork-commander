#include "atwork_commander_com/plugin_interface.h"

#include <pluginlib/class_list_macros.h>

namespace atwork_commander {
namespace com_plugin {
namespace ros {

class TaskArenaCentric : public Base {

    ::ros::NodeHandle rh;

    ::ros::Subscriber robot_state_sub;
    ::ros::Publisher send_task_pub;

public:
    TaskArenaCentric() {}
    virtual ~TaskArenaCentric() {}

    virtual void sendTask( atwork_commander_msgs::Task task ) {
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

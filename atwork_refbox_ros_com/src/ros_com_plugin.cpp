#include "atwork_refbox_com/plugin_interface.h"

#include <pluginlib/class_list_macros.h>

namespace atwork_refbox_ros {
namespace communication {

class RosCom : public Interface {

    ros::NodeHandle rh;

    ros::Subscriber robot_state_sub;
    ros::Publisher send_task_pub;

public:
    RosCom() {}
    virtual ~RosCom() {}

    virtual void sendTask( atwork_refbox_ros_msgs::Task task ) {
        this->send_task_pub.publish( task );
    }

    virtual void onInit( ros::NodeHandle roshandle ) {
        this->rh = roshandle;

        this->robot_state_sub = this->rh.subscribe( "/refbox/robot_state", 10, &RosCom::receiveRobotStateClb, this );
    	this->send_task_pub = this->rh.advertise<atwork_refbox_ros_msgs::Task>("/refbox/task", 1);
    }

private:
    void receiveRobotStateClb( const atwork_refbox_ros_msgs::RobotState::ConstPtr& msg ) {
    	this->sendRobotState( *msg );
    }

};

}; // ns atwork_refbox_ros
}; // ns communication


PLUGINLIB_EXPORT_CLASS(atwork_refbox_ros::communication::RosCom, atwork_refbox_ros::communication::Interface)

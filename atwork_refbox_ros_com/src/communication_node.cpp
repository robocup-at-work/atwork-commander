#include "atwork_refbox_com/plugin_interface.h"
#include <pluginlib/class_loader.h>

ros::Publisher robot_state_pub;

boost::shared_ptr<atwork_refbox_ros::com_plugin::ComInterface> plugin;


void sendTaskClb( const atwork_refbox_ros_msgs::Task::ConstPtr& msg ) {
	plugin->sendTask( *msg );
}

void receiveRobotState( atwork_refbox_ros_msgs::RobotState robot_state ) {
	ROS_INFO_STREAM("new robot_state: " << robot_state );
	robot_state_pub.publish( robot_state );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "atwork_refbox_com_node");
	ros::NodeHandle roshandle;

	ros::Subscriber send_task_sub = roshandle.subscribe( "/refbox/internal/task", 1, &sendTaskClb );
	robot_state_pub = roshandle.advertise<atwork_refbox_ros_msgs::RobotState>("/refbox/internal/robot_state", 10);

	//TODO: list, no default
	std::string plugin_name;
	//the plugin_name should include the namespace
	ros::param::param<std::string>("~plugin", plugin_name, "atwork_refbox_ros::com_plugin::null");

	pluginlib::ClassLoader<atwork_refbox_ros::com_plugin::ComInterface> plug_in_loader( "atwork_refbox_com", "atwork_refbox_ros::com_plugin::ComInterface" );

	ROS_INFO("[main] starting plugin \"%s\"", plugin_name.c_str());
	try
	{
		plugin.reset();
		plugin = plug_in_loader.createInstance(plugin_name.c_str());
		plugin->initialize(roshandle, std::bind( &receiveRobotState, std::placeholders::_1 ));
	} catch(pluginlib::PluginlibException& ex) {
		ROS_FATAL("[main] failed to load. Error: \"%s\"\nGoodbye.", ex.what());
		return 1;
	}
	ROS_INFO("[main] module loaded");

	// plugin->initialize(roshandle);

	ros::spin();

	plugin.reset();

	return 0;
}

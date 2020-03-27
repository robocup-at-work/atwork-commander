#include "atwork_commander_com/plugin_interface.h"
#include <pluginlib/class_loader.h>

using if_t = atwork_commander::communication::Interface;

ros::Publisher g_robot_state_pub;
std::vector<boost::shared_ptr<if_t>> g_plugins;

void sendTaskClb(const atwork_commander_msgs::Task::ConstPtr& msg)
{
    for (auto& p : g_plugins) {
        p->sendTask(*msg);
    }
}

void receiveRobotState(atwork_commander_msgs::RobotState robot_state)
{
    // ROS_INFO_STREAM("new robot_state: " << robot_state);
    g_robot_state_pub.publish(robot_state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "communications_hub");
    ros::NodeHandle roshandle;

    ros::Subscriber send_task_sub = roshandle.subscribe("internal/task", 1, &sendTaskClb);
    g_robot_state_pub = roshandle.advertise<atwork_commander_msgs::RobotState>("internal/robot_state", 10);

    std::vector<std::string> plugins_param;
    ros::param::get("~plugins", plugins_param);

    pluginlib::ClassLoader<if_t> plug_in_loader("atwork_commander_com", "atwork_commander::communication::Interface");

    for (auto p : plugins_param) {
        try {
            g_plugins.emplace_back(plug_in_loader.createInstance(p.c_str()));
            g_plugins.back()->initialize(roshandle, std::bind(&receiveRobotState, std::placeholders::_1));
            ROS_INFO("[main|com] successfully initialized plugin \"%s\"", p.c_str());
        } catch (pluginlib::PluginlibException& ex) {
            ROS_FATAL("[main|com] failed to load plugin \"%s\". Error:\n\"%s\"", p.c_str(), ex.what());
        }
    }

    if (g_plugins.size() > 0) {
        ROS_INFO("[main|com] plugin loading finished");
    } else {
        ROS_FATAL("[main|com] no communication plugin was loaded. Exiting");
        return 1;
    }

    ros::spin();

    for (auto& p : g_plugins) {
        std::cout << p.use_count();
        p.reset();
    }

    return 0;
}

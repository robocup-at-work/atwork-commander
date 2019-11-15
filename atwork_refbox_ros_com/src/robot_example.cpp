#include "ros/ros.h"

#include "atwork_refbox_ros_msgs/RobotState.h"
#include "atwork_refbox_ros_msgs/Task.h"

atwork_refbox_ros_msgs::Task g_cur_task;
atwork_refbox_ros_msgs::RobotState g_robot_state;

void taskClb(const atwork_refbox_ros_msgs::Task::ConstPtr& msg)
{
    g_cur_task = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atwork_refbox_com_robot_example_node");
    ros::NodeHandle roshandle;

    ros::Subscriber task_sub = roshandle.subscribe("/refbox/task", 1, &taskClb);
    ros::Publisher robot_state_pub = roshandle.advertise<atwork_refbox_ros_msgs::RobotState>("/refbox/robot_state", 1);

    g_robot_state.sender.team_name = "robOTTO";
    g_robot_state.sender.robot_name = "Euler";

    ros::Duration d(0.5);

    while ( ros::ok() )
    {
        g_robot_state.sender.header.stamp = ros::Time::now();
        //TODO change Task periodically

        robot_state_pub.publish( g_robot_state );
        ros::spinOnce();
        d.sleep();
    }

    return 0;
}

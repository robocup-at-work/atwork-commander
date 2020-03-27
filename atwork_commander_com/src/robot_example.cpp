#include "ros/ros.h"

#include "atwork_commander_msgs/RobotState.h"
#include "atwork_commander_msgs/Task.h"

atwork_commander_msgs::Task g_cur_task;
atwork_commander_msgs::RobotState g_robot_state;

void taskClb(const atwork_commander_msgs::Task::ConstPtr& msg)
{
    g_cur_task = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_fake_robot");
    ros::NodeHandle roshandle;

    ros::Subscriber task_sub = roshandle.subscribe("/refbox/task", 1, &taskClb);
    ros::Publisher robot_state_pub = roshandle.advertise<atwork_commander_msgs::RobotState>("/refbox/robot_state", 1);

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

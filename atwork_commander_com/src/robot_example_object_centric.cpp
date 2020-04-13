#include "ros/ros.h"

#include "atwork_commander_msgs/RobotState.h"
#include "atwork_commander_msgs/ObjectTask.h"

#include <string>


void taskClb(const atwork_commander_msgs::ObjectTask::ConstPtr& msg)
{
    ROS_INFO_STREAM_NAMED("example", "[REFBOX_EXAMPLE] Got current task:" << std::endl << *msg );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_fake_robot");
    ros::NodeHandle nh("~");

    std::string refboxName = "atwork_commander";
    std::string teamName = "exampleTeam";
    std::string robotName = "exampleRobot";

    if( !nh.getParam("refboxName", refboxName) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No refbox name configured using default: \"atwork_commander\"!");
    if( !nh.getParam("teamName", teamName) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No team name configured using default: \"exampleTeam\"!");
    if( !nh.getParam("robotName", robotName) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No robot name configured using default: \"exampleRobot\"!");

    ros::Subscriber task_sub = nh.subscribe(std::string("/")+refboxName+"/object_task", 1, &taskClb);
    ros::Publisher robot_state_pub = nh.advertise<atwork_commander_msgs::RobotState>(std::string("/")+refboxName+"/robot_state", 1);

    atwork_commander_msgs::RobotState robot_state;
    robot_state.sender.team_name = teamName;
    robot_state.sender.robot_name = robotName;

    ros::Duration d(0.5);

    while ( ros::ok() )
    {
        robot_state.sender.header.stamp = ros::Time::now();
        //TODO change Task periodically

        robot_state_pub.publish( robot_state );
        ros::spinOnce();
        d.sleep();
    }

    return 0;
}

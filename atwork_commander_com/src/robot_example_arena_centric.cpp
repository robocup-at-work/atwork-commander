#include "ExampleReportGenerator.h"

#include "ros/ros.h"

#include "atwork_commander_msgs/RobotState.h"
#include "atwork_commander_msgs/Task.h"

#include <string>
#include <iostream>

atwork_commander_msgs::RobotState gRobot_state;
ros::Publisher                    gRobot_state_pub;
atwork_commander_msgs::Task       gTask;

ExampleReportGenerator gen(gRobot_state, gTask);

void taskClb(const atwork_commander_msgs::Task::ConstPtr& msg) {
    ROS_INFO_STREAM_THROTTLE_NAMED(60, "example", "[REFBOX_EXAMPLE] Got current task:" << std::endl << *msg );
    gTask = *msg;
}

void update(const ros::TimerEvent& e) {
  gen.update();
}

void report(const ros::TimerEvent& e) {
  gRobot_state_pub.publish( gRobot_state );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_fake_robot");
    ros::NodeHandle nh("~");

    std::string refboxName = "atwork_commander";
    std::string teamName   = "exampleTeam";
    std::string robotName  = "exampleRobot";
    float       reportFreq = 2.0;
    float       updateFreq = 0.1;

    if( !nh.getParam("refboxName", refboxName) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No refbox name configured using default: \"atwork_commander\"!");
    if( !nh.getParam("teamName", teamName) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No team name configured using default: \"exampleTeam\"!");
    if( !nh.getParam("robotName", robotName) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No robot name configured using default: \"exampleRobot\"!");
    if( !nh.getParam("reportFreq", reportFreq) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No report frequency configured using default: \"2.0 Hz\"!");
    if( !nh.getParam("updateFreq", updateFreq) )
      ROS_WARN_STREAM_NAMED("example", "[REFBOX-EXAMPLE] No update frequency configured using default: \"0.1 Hz\"!");

    gRobot_state.sender.team_name = teamName;
    gRobot_state.sender.robot_name = robotName;

    ros::Subscriber task_sub         = nh.subscribe(std::string("/")+refboxName+"/task", 1, &taskClb);
                    gRobot_state_pub = nh.advertise<atwork_commander_msgs::RobotState>(std::string("/")+refboxName+"/robot_state", 1);
    ros::Timer      reportTimer      = nh.createTimer(ros::Duration(1.0/reportFreq), report);
    ros::Timer      updateTimer      = nh.createTimer(ros::Duration(1.0/updateFreq), update);


    ROS_INFO_STREAM_NAMED("example", "[REFBOX-EXAMPLE] Example Robot started!");

    ros::spin();

    return 0;
}

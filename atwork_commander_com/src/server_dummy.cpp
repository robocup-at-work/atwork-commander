#include "ros/ros.h"

#include "atwork_commander_msgs/RobotState.h"
#include "atwork_commander_msgs/Task.h"

void robotStateClb(const atwork_commander_msgs::RobotState::ConstPtr& msg)
{
    ROS_INFO_STREAM("new robot_state: " << *msg);
}

atwork_commander_msgs::Task createDummyTask() {
    atwork_commander_msgs::RobotHeader robot;
    robot.team_name = "robOTTO";
    robot.robot_name = "Euler";

    atwork_commander_msgs::Object m20;
    m20.object = atwork_commander_msgs::Object::M20;
    atwork_commander_msgs::Object f20g;
    f20g.object = atwork_commander_msgs::Object::F20_20_G;
    atwork_commander_msgs::Object bearing;
    bearing.object = atwork_commander_msgs::Object::BEARING;
    atwork_commander_msgs::Object motor;
    motor.object = atwork_commander_msgs::Object::MOTOR;
    atwork_commander_msgs::Object s40b;
    s40b.object = atwork_commander_msgs::Object::S40_40_B;

    atwork_commander_msgs::Workstation w_s_1;
    w_s_1.name = "WS01";
    w_s_1.objects.push_back(m20);
    w_s_1.objects.push_back(f20g);
    w_s_1.objects.push_back(bearing);
    w_s_1.objects.push_back(motor);
    atwork_commander_msgs::Workstation w_s_2;
    w_s_2.name = "WS02";
    w_s_2.objects.push_back(m20);
    w_s_2.objects.push_back(s40b);
    atwork_commander_msgs::Workstation w_t_2;
    w_t_2.name = "WS02";
    w_t_2.objects.push_back(bearing);
    w_t_2.objects.push_back(f20g);
    w_t_2.objects.push_back(motor);
    atwork_commander_msgs::Workstation w_t_3;
    w_t_3.name = "WS03";
    w_t_3.objects.push_back(m20);
    w_t_3.objects.push_back(m20);
    w_t_3.objects.push_back(s40b);

    atwork_commander_msgs::Task task;
    task.execute_on.push_back(robot);
    task.arena_start_state.push_back(w_s_1);
    task.arena_start_state.push_back(w_s_2);
    task.arena_target_state.push_back(w_t_2);
    task.arena_target_state.push_back(w_t_3);

    return task;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_server");
    ros::NodeHandle roshandle;

    ros::Subscriber robot_state_sub = roshandle.subscribe("internal/robot_state", 1, &robotStateClb);
    ros::Publisher send_task_pub = roshandle.advertise<atwork_commander_msgs::Task>("internal/task", 1);

    auto task = createDummyTask();
    ros::Duration d(2);

    while ( ros::ok() )
    {
        send_task_pub.publish( task );
        ros::spinOnce();
        d.sleep();
    }

    return 0;
}

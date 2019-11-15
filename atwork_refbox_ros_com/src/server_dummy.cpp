#include "ros/ros.h"

#include "atwork_refbox_ros_msgs/RobotState.h"
#include "atwork_refbox_ros_msgs/Task.h"

void robotStateClb(const atwork_refbox_ros_msgs::RobotState::ConstPtr& msg)
{
    ROS_INFO_STREAM("new robot_state: " << *msg);
}

atwork_refbox_ros_msgs::Task createDummyTask() {
    atwork_refbox_ros_msgs::RobotHeader robot;
    robot.team_name = "robOTTO";
    robot.robot_name = "Euler";

    atwork_refbox_ros_msgs::Object m20;
    m20.object = atwork_refbox_ros_msgs::Object::M20;
    atwork_refbox_ros_msgs::Object f20g;
    f20g.object = atwork_refbox_ros_msgs::Object::F20_20_G;
    atwork_refbox_ros_msgs::Object bearing;
    bearing.object = atwork_refbox_ros_msgs::Object::BEARING;
    atwork_refbox_ros_msgs::Object motor;
    motor.object = atwork_refbox_ros_msgs::Object::MOTOR;
    atwork_refbox_ros_msgs::Object s40b;
    s40b.object = atwork_refbox_ros_msgs::Object::S40_40_B;

    atwork_refbox_ros_msgs::Workstation w_s_1;
    w_s_1.workstation_name = "WS01";
    w_s_1.objects.push_back(m20);
    w_s_1.objects.push_back(f20g);
    w_s_1.objects.push_back(bearing);
    w_s_1.objects.push_back(motor);
    atwork_refbox_ros_msgs::Workstation w_s_2;
    w_s_2.workstation_name = "WS02";
    w_s_2.objects.push_back(m20);
    w_s_2.objects.push_back(s40b);
    atwork_refbox_ros_msgs::Workstation w_t_2;
    w_t_2.workstation_name = "WS02";
    w_t_2.objects.push_back(bearing);
    w_t_2.objects.push_back(f20g);
    w_t_2.objects.push_back(motor);
    atwork_refbox_ros_msgs::Workstation w_t_3;
    w_t_3.workstation_name = "WS03";
    w_t_3.objects.push_back(m20);
    w_t_3.objects.push_back(m20);
    w_t_3.objects.push_back(s40b);

    atwork_refbox_ros_msgs::Task task;
    task.execute_on.push_back(robot);
    task.arena_start_state.push_back(w_s_1);
    task.arena_start_state.push_back(w_s_2);
    task.arena_target_state.push_back(w_t_2);
    task.arena_target_state.push_back(w_t_3);

    return task;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atwork_refbox_com_server_dummy_node");
    ros::NodeHandle roshandle;

    ros::Subscriber robot_state_sub = roshandle.subscribe("/refbox/internal/robot_state", 1, &robotStateClb);
    ros::Publisher send_task_pub = roshandle.advertise<atwork_refbox_ros_msgs::Task>("/refbox/internal/task", 1);

    auto task = createDummyTask();
    ros::Duration d(2);

    while ( ros::ok() )
    {
        for( auto& r : task.execute_on ) {
            r.header.stamp = ros::Time::now();
        }
        send_task_pub.publish( task );
        ros::spinOnce();
        d.sleep();
    }

    return 0;
}

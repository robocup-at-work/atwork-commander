
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "atwork_refbox_msgs/tasklist.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
using namespace std;

	
vector<string> myvector;

bool tasklist(atwork_refbox_msgs::tasklist::Request  &req,
         atwork_refbox_msgs::tasklist::Response &res)
{
  res.tasklist = myvector;
  //ROS_INFO_STREAM(myvector);
  return true;

}


int main(int argc, char **argv)

{
		ros::init(argc, argv, "topic");
  		ros::NodeHandle st_viz_task_handler;
 		ros::Publisher st_viz_task_talker = st_viz_task_handler.advertise<std_msgs::Int32MultiArray>("st_viz_task_topic", 1);
 		ros::Rate loop_rate(1);
			
			myvector.push_back("BNT");

			ros::ServiceServer service = st_viz_task_handler.advertiseService("ListofTasks", tasklist);

  			ros::spin();

return 0;

}	



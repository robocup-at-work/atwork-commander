#include <ros/ros.h>
#include "receiver_node_atc.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "receiver_node");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(10); // one Hz

	ReceiverNode node(nh);

	ROS_INFO("robOTTO RefBox Client running");
		
	try {
		while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	
	catch(int error) {
		switch (error) {
			case 100 : ROS_ERROR_STREAM(error<<"Undefined discipline");
			case 200 : ROS_ERROR_STREAM(error<<"[BTT3] No objects: Can't generate tasks without objects");
			case 201 : ROS_ERROR_STREAM(error<<"[BTT3] No shelfs: Can't generate shelf tasks without shelf");
			case 202 : ROS_ERROR_STREAM(error<<"[BTT3] No tables: Can't generate more tasks than pick_shelfs without tables");
			case 203 : ROS_ERROR_STREAM(error<<"[BTT3] infeasible constraints for task creation: place = pick");
			case 204 : ROS_ERROR_STREAM(error<<"[BTT3] No tables, no shelfs: Can't place Container in air");
			case 205 : ROS_ERROR_STREAM(error<<"[BTT3] No tables: Can't place Container in air");
			case 210 : ROS_ERROR_STREAM(error<<"[BNT] No waypoints");
			case 211 : ROS_ERROR_STREAM(error<<"[BNT] No tables");
			case 220 : ROS_ERROR_STREAM(error<<"[Final] No objects");
			case 221 : ROS_ERROR_STREAM(error<<"[Final] No shelfs: Can't generate shelf tasks without shelf");
			case 222 : ROS_ERROR_STREAM(error<<"[Final] No tables: Can't generate more tasks than pick_shelfs without tables");
			case 223 : ROS_ERROR_STREAM(error<<"[Final] infeasible constraints for task creation: place = pick");
			case 224 : ROS_ERROR_STREAM(error<<"[Final] No tables, no shelfs: Can't place Container in air");
			case 225 : ROS_ERROR_STREAM(error<<"[Final] No tables: Can't place Container in air");
			case 226 : ROS_ERROR_STREAM(error<<"[Final] No cavity plattforms: Can't generate cavity plattform tasks without cavity plattforms");
			case 227 : ROS_ERROR_STREAM(error<<"[Final] No conveyers: Can't generate conveyer tasks without conveyers");
			case 228 : ROS_ERROR_STREAM(error<<"[Final] No valid object for PPT");
			default  : ROS_ERROR_STREAM(error<<"Unknown error");
		}
	}
	

  return 0;
}

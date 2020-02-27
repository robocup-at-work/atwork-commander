#include <ros/ros.h>
#include <atwork_refbox_ros_task_generator/receiver_node_atc.h>

using atwork_refbox_ros::ReceiverNode;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "receiver_node");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(10); //  Hz

		
	try {
    ReceiverNode node(nh);

    ROS_INFO("robOTTO RefBox Client running");
		while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	
	catch(int error) {
		switch (error) {
			case 100 : ROS_ERROR_STREAM(error<<"Undefined discipline"); break;
			case 200 : ROS_ERROR_STREAM(error<<"[BTT3] No objects: Can't generate tasks without objects"); break;
			case 201 : ROS_ERROR_STREAM(error<<"[BTT3] No shelfs: Can't generate shelf tasks without shelf"); break;
			case 202 : ROS_ERROR_STREAM(error<<"[BTT3] No tables: Can't generate more tasks than pick_shelfs without tables"); break;
			case 203 : ROS_ERROR_STREAM(error<<"[BTT3] infeasible constraints for task creation: place = pick"); break;
			case 204 : ROS_ERROR_STREAM(error<<"[BTT3] No tables, no shelfs: Can't place Container in air"); break;
			case 205 : ROS_ERROR_STREAM(error<<"[BTT3] No tables: Can't place Container in air"); break;
			case 210 : ROS_ERROR_STREAM(error<<"[BNT] No waypoints"); break;
			case 211 : ROS_ERROR_STREAM(error<<"[BNT] No tables"); break;
			case 220 : ROS_ERROR_STREAM(error<<"[Final] No objects"); break;
			case 221 : ROS_ERROR_STREAM(error<<"[Final] No shelfs: Can't generate shelf tasks without shelf"); break;
			case 222 : ROS_ERROR_STREAM(error<<"[Final] No tables: Can't generate more tasks than pick_shelfs without tables"); break;
			case 223 : ROS_ERROR_STREAM(error<<"[Final] infeasible constraints for task creation: place = pick"); break;
			case 224 : ROS_ERROR_STREAM(error<<"[Final] No tables, no shelfs: Can't place Container in air"); break;
			case 225 : ROS_ERROR_STREAM(error<<"[Final] No tables: Can't place Container in air"); break;
			case 226 : ROS_ERROR_STREAM(error<<"[Final] No cavity plattforms: Can't generate cavity plattform tasks without cavity plattforms"); break;
			case 227 : ROS_ERROR_STREAM(error<<"[Final] No conveyers: Can't generate conveyer tasks without conveyers"); break;
			case 228 : ROS_ERROR_STREAM(error<<"[Final] No valid object for PPT"); break;
			case 229 : ROS_ERROR_STREAM(error<<"[Final] Unknown color of container"); break;
			case 230 : ROS_ERROR_STREAM(error<<"[Final] No valid picks left"); break;
			case 231 : ROS_ERROR_STREAM(error<<"[Final] No tables0 : Can't generate table0 picks without table0"); break;
			case 232 : ROS_ERROR_STREAM(error<<"[Final] No tables5 : Can't generate table0 picks without table5"); break;
			case 233 : ROS_ERROR_STREAM(error<<"[Final] No tables10 : Can't generate table0 picks without table10"); break;
			case 234 : ROS_ERROR_STREAM(error<<"[Final] No tables15 : Can't generate table0 picks without table15"); break;
			default  : ROS_ERROR_STREAM(error<<"Unknown error");
		}
	}
	

  return 0;
}

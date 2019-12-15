#include <atwork_refbox_ros_task_generator/TaskGenerator.h>


using namespace atwork_refbox_ros;
using namespace std;

TaskGenerator::TaskGenerator(Options globalOptions, TaskDefinitions tasks, Workstations workstations) 
  : mTasks(tasks), mNode(globalOptions, tasks, workstations)
{

}

Task TaskGenerator::operator()(string name) {
  try {
    unsigned int objects=0;
    unsigned int tables=0;
    unsigned int decoys=0;
    unsigned int pickShelf=0;
    unsigned int pickTT=0;
    unsigned int placeShelf=0;
    unsigned int placeTT=0;
    unsigned int placePPT=0;
    unsigned int containerRed=0;
    unsigned int containerBlue=0;
    mNode.readParameters(objects, tables, decoys, pickShelf, pickTT, placeShelf, placeTT, placePPT, containerRed, containerBlue);
  }catch(int error){
		switch (error) {
			case 100 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"Undefined discipline");
			case 200 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BTT3] No objects: Can't generate tasks without objects");
			case 201 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BTT3] No shelfs: Can't generate shelf tasks without shelf");
			case 202 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BTT3] No tables: Can't generate more tasks than pick_shelfs without tables");
			case 203 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BTT3] infeasible constraints for task creation: place = pick");
			case 204 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BTT3] No tables, no shelfs: Can't place Container in air");
			case 205 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BTT3] No tables: Can't place Container in air");
			case 210 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BNT] No waypoints");
			case 211 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[BNT] No tables");
			case 220 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No objects");
			case 221 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No shelfs: Can't generate shelf tasks without shelf");
			case 222 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No tables: Can't generate more tasks than pick_shelfs without tables");
			case 223 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] infeasible constraints for task creation: place = pick");
			case 224 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No tables, no shelfs: Can't place Container in air");
			case 225 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No tables: Can't place Container in air");
			case 226 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No cavity plattforms: Can't generate cavity plattform tasks without cavity plattforms");
			case 227 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No conveyers: Can't generate conveyer tasks without conveyers");
			case 228 : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"[Final] No valid object for PPT");
			default  : ROS_ERROR_STREAM_NAMED("[REFBOX]", error<<"Unknown error");
		};
    return Task();
  }catch(const std::exception& e){
    ROS_ERROR_STREAM_NAMED("[REFBOX]", "No task named " << name);
    return Task();
  }
  auto task = mNode.generate_Final();
  return Task();
}

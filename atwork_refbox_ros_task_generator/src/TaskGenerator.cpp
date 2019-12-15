#include <atwork_refbox_ros_task_generator/TaskGenerator.h>


using namespace atwork_refbox_ros;
using namespace std;

TaskGenerator::TaskGenerator(const ArenaDescription& arena, const TaskDefinitions& tasks) 
  : mTasks(tasks), mNode(arena, tasks)
{

}

Task TaskGenerator::operator()(string name) {
  try {
    auto& params = mTasks[name];
    unsigned int objects=params["object_count"];
    unsigned int tables=params["table_height_0"]+params["table_height_5"]+params["table_height_10"]+params["table_height_15"];
    unsigned int decoys=params["decoy_count"];
    unsigned int pickShelf=params["shelves_grasping"];
    unsigned int pickTT=params["rt_grasping"];
    unsigned int placeShelf=params["shelves_placing"];
    unsigned int placeTT=params["rt_placing"];
    unsigned int placePPT=params["pp"];
    unsigned int containerRed=params["container_red"];
    unsigned int containerBlue=params["container_blue"];
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

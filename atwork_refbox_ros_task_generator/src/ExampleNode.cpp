#include <atwork_refbox_ros_task_generator/TaskGenerator.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <exception>

using namespace atwork_refbox_ros;
using namespace std;

static void activateDebug() {
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();
  ros::Duration(1).sleep(); // Necessary to allow rosconsole to react to logger level change
}

static auto exampleTaskDef() {
    TaskDefinition example{
      {"waypoint_count",0},
      {"object_count",5},
      {"decoy_count",3},
      {"pp",1},
      {"rt_grasping",1},
      {"rt_placing",1},
      {"shelfes_grasping",2},
      {"shelfes_placing",2},
      {"table_height_0", 1},
      {"table_height_5", 1},
      {"table_height_10", 1},
      {"table_height_15", 1},
      {"F20_20_G",1},
      {"F20_20_B",1},
      {"S40_40_B",1},
      {"S40_40_G",1},
      {"M20",1},
      {"M30",1},
      {"M20_100",1},
      {"R20",1},
      {"AXIS",1},
      {"BEARING",1},
      {"BEARING_BOX",1},
      {"DISTANCE_TUBE",1},
      {"MOTOR",1},
      {"container_red",1},
      {"container_blue",1}
    };
    TaskDefinitions def{{"EXAMPLE", example}};
    return def;
}

static auto exampleArena() {
    ArenaDescription arena;
    arena.cavities=decltype(arena.cavities){
      {"F20_20_H", true},
      {"F20_20_V",false},
      {"S40_40_H",true},
      {"S40_40_V",false},
      {"M20_H",true},
      {"M20_V",true},
      {"M30_H",true},
      {"M30_V",true},
      {"M20_100_H",true},
      {"R20_H",true},
      {"R20_V",true}
    },
    arena.workstations=decltype(arena.workstations){
      {"WS01","10"},
      {"WS02","10"},
      {"WS03","05"},
      {"WS04","00"},
      {"WS05","00"},
      {"WS06","15"},
      {"WS07","15"},
      {"WS08","05"},
      {"PP01","PP"},
      {"TT01","TT"},
      {"SH01","SH"},
      {"SH02","SH"}
    };
  return arena;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_task_generator");
  ros::NodeHandle nh("~");
  activateDebug();

  auto def = exampleTaskDef();
  auto arena = exampleArena();
  ROS_DEBUG_STREAM_NAMED("example", "[REFBOX-GEN] " << arena);
  ROS_DEBUG_STREAM_NAMED("example", "[REFBOX-GEN] " << def);

  try {
    TaskGenerator gen(arena, def);
    Task task = gen("EXAMPLE");
    ROS_DEBUG_STREAM_NAMED("example", "[REFBOX-GEN] Tasks:\n" << task);
  }
  catch(exception& e) {
    ROS_ERROR_STREAM_NAMED("example", "[REFBOX-GEN] Exception occured: \n" << e.what());
  }

  while(ros::ok())
    ros::spin();
  return 0;
}

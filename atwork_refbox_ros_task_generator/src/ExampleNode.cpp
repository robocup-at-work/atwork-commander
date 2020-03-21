#include <atwork_refbox_ros_task_generator/TaskGenerator.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <exception>
#include <thread>
#include <chrono>

using namespace atwork_refbox_ros;
using namespace std;
using namespace chrono;

static void activateDebug() {
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
    this_thread::sleep_for(seconds(1)); // Necessary to allow rosconsole to react to logger level change
  }
}

static auto exampleTaskDef() {
    TaskDefinition example;
    example.parameters = ParameterType {
      { "seed"               , 1234},
      { "prep_time"          , 3},
      { "exec_time"          , 8},
      { "waypoints"          , 0},
      { "objects"            , 5},
      { "decoys"             , 3},
      { "pp_placing"         , 1},
      { "ref_cavity_orientation", 1},
      { "container_placing"  , 2},
      { "container_in_shelf" , 0},
      { "container_on_rt"    , 0},
      { "tt_grasping"        , 1},
      { "tt_placing"         , 1},
      { "shelfes_grasping"   , 2},
      { "shelfes_placing"    , 2}
    };
    example.objects = ParameterType {
      { "F20_20_G"           , 1},
      { "F20_20_B"           , 1},
      { "S40_40_B"           , 1},
      { "S40_40_G"           , 1},
      { "M20"                , 1},
      { "M30"                , 1},
      { "M20_100"            , 1},
      { "R20"                , 1},
      { "AXIS"               , 1},
      { "BEARING"            , 1},
      { "BEARING_BOX"        , 1},
      { "DISTANCE_TUBE"      , 1},
      { "MOTOR"              , 1},
      { "CONTAINER_RED"      , 1},
      { "CONTAINER_BLUE"     , 1}
    };
    TaskDefinitions def{{"EXAMPLE", example}};
    return def;
}

static auto exampleArena() {
    ArenaDescription arena;
    arena.cavities=decltype(arena.cavities){
      "F20_20_H",
      "F20_20_V",
      "S40_40_H",
      "S40_40_V",
      "M20_H",
      "M20_V",
      "M30_H",
      "M30_V",
      "M20_100_H",
      "R20_H",
      "R20_V"
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
    arena.objects=decltype(arena.objects){
      { "F20_20_G"      , 1},
      { "F20_20_B"      , 1},
      { "S40_40_B"      , 1},
      { "S40_40_G"      , 1},
      { "M20"           , 1},
      { "M30"           , 1},
      { "M20_100"       , 1},
      { "R20"           , 1},
      { "AXIS"          , 1},
      { "BEARING"       , 1},
      { "BEARING_BOX"   , 1},
      { "DISTANCE_TUBE" , 1},
      { "MOTOR"         , 1},
      { "CONTAINER_RED" , 1},
      { "CONTAINER_BLUE", 1}
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

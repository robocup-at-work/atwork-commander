# atwork\_commander\_gen

Enable dynamic task generation according to specified task types of the [@Work
RuleBook](https://github.com/robocup-at-work/rulebook). 

## Architecture

TaskGenerator with pluggable generators and configuration infrastructure.

The ```TaskGenerator``` class if the interface to embed the functionality in other programs. The class uses
ros-parameters to extract configuration information including the Arena and Tasks-Specification as well as the genrator
plugin to be used.

The generator plugins are implemented using ROS pluginlib. The implementation currently uses pluginlib, logging and
the parameter-server API of ROS.

The ```TaskGenerator``` is no ROS-Node, but a library to be used in other ROS-Nodes like the refbox state-machine from
the ```atwork_commander_core```.

__TODO__: Insert Overview-Image

## Generators

There are currently two generators implemented:

* ```Simple```:
  * Simple task generator to be used as basis for own creations
  * __Not Rule-Book compatible__
* ```JurekGen```:
  * Generator created by _Jurek Rostalsky_ from the _robOTTO_ Team with additional integration code
  * __Rule-Book compatible__
  * Because of additional integration code currenlty messed up software architecture
  * Hardly maintanable

Generators can be selected through the launch parameter ```generator``` in 
```atwork_commander atwork_commander.launch``` and ```atwork_commander_gen example.launch```

## Configuration

Uses simple ROS parameter-based specification of the task types and the arena, which is used to generate task
instances accordingly. The general approach is to specify the configuration using ```.yaml``` files, which are either
manually loaded using ```rosparam load``` or automatically loaded within ```.launch``` files.

### Arena Configuration

The arena, where the task are executed in, is specified as a hierarchical parameter in the parameter
server. The name of the this parameter is passed on ```TaskGenerator``` construction through the ```arenaConfig```
parameter. The arena parameter contains the following sub-parts.

By default the parameter ```arena``` is used to read the arena specification from in the launch files 
```atwork_commander atwork_commander.launch``` and ```atwork_commander_gen example.launch```.

An example arena configuration can be found in ```atwork_commander_gen/config/example-arena.yaml```

#### Workstations

The ```workstations``` sub-parameter contains individual key-value pairs of workstation names and the according
workstation type ```<ws-name>: "<ws-type>"```. An example Workstation specification looks like  this:

```
workstations:
    WS01: "10"
    WS02: "15"
    WS03: "05"
    WS04: "00"
    WS06: "15"
    WS08: "05"
    PP01: "PP"
    TT01: "TT"
    SH02: "SH"
```

__TODO__: This will be extended in the future to also include pose information of the workstations for rendering 
purposes

Currently the following workstation types are implemented:

Type      | Description
----------|-------------
```00```  |  0cm height Workstation
```05```  |  5cm height Workstation
```10```  | 10cm height Workstation (default)
```15```  | 15cm height Workstaton
```PP```  | Precision Placement Workstation (10cm by default)
```TT```  | Rotating Table or Conveyor Belt (10cm height by default)
```CBT``` | Rotating Table or Conveyor Belt (10cm height by default)


#### Cavities

Available cavities in the arena are specified as a list of cavity names e.g.: 

```
cavities: [ "F20_20_H", "S40_40_H", "S40_40_V", "M20_H", "M20_V", "M30_H", "M30_V", "M20_100_H", "R20_H", "R20_V" 
```

Each cavitiy description consists of the form part e.g. ```F20_20``` for small aluminium profile and a specifier for
the orientation. The orientation may either be ```_H``` for horizontal placement or ```_V``` for vertical placement.
The reference face for this  placement is always the face with the largest area.

The task generation will only generate tasks wich use one of these cavities (__TODO__: Not yet implemented for JurekGen) 

#### Objects

The object specification describes the available objects of each type for tasks executed in the arena. It contains
key-value pairs of ```<object-type>: <count>``` entries in the sub-parameter ```objects```.

The idea is to enable the task generator to only use up as many objects of each type as available 
(__TODO__: not yet implemented for JurekGen)

Example object specification:

```
objects:
  F20_20_G      : 1
  F20_20_B      : 1
  S40_40_B      : 1
  S40_40_G      : 1
  M20           : 1
  M30           : 1
  M20_100       : 1
  R20           : 1
  AXIS          : 1
  BEARING       : 1
  BEARING_BOX   : 1
  DISTANCE_TUBE : 1
  MOTOR         : 1
  CONTAINER_RED : 1
  CONTAINER_BLUE: 1
```

### Task Configuration

Task configuration consists of multiple sub-specifications indicating different aspects of the task. The different
configuration options will be sorted based on their data type.

All configurations options are sub-parameters of a task. Each task uses the task name as thesub-parameter name. The
```TaskGenerator```'s second options taskConfig describes the parameter, which contains the individual task types as
sub-parameters. Typically this parameter is ```tasks```.

The possible sub-parameters of each task have a default parameter, which is used if the parameter is not specified. If
parameters are specified that are unknown to the config parser a warning will be printed, but no error is raised.

The default values are specified in `atwork_commander_gen/include/atwork_commander_gen/ConfigParserInterface.h`

#### Integer Configurations

Most of the configuration parameters of each task are integer variables. Boolean choices are also represented through
integer variables. In the following there are all the currently supported integer variables described.

Name                   | Range          | Default | Description
-----------------------|----------------|---------|------------
prep_time              | 1 - 2147483647 |  1      | Time in minutes for the preparation phase
exec_time              | 1 - 2147483647 |  1      | Time in minutes for the execution phase
waypoints              | 0 - 2147483647 |  0      | Number of waypoints to generate (Not implemented for JurekGen)
objects                | 0 - 2147483647 |  1      | Number of objects to transport
decoys                 | 0 - 2147483647 |  0      | Number of decoys to place on workstations
barrier_tapes          | 0 - 2147483647 |  0      | Number of barrier tapes to be placed in the arena (Not yet implemented)
obstacles              | 0 - 2147483647 |  0      | Number of obstacles to be placed in the arena (Not yet implemented)
arbitrary_surfaces     | 0 - 2147483647 |  0      | number of workstations, where objects are picked from, which contain arbitrary surfaces (not yet implemented)
tables                 | 1 - 2147483647 |  0      | number of workstations to be used in task (not yet implemented for JurekGen)
tables                 | 
pp_placing             | 0 - 2147483647 |  0      | Number of objects to be placed on a PPT in the appropriate cavity
ref_cavity_position    | 0 - 1          |  0      | Boolean if cavity position  is decided by referees (Ignored, currently assumed to be always 1)
ref_cavity_rotation    | 0 - 1          |  0      | Boolean if cavity rotation is decided by referees (Not yet implemented)
ref_cavity_orientation | 0 - 1          |  0      | Boolean if cavity orientation is decided by referees (Not implemented in JurekGen)
container_placing      | 0 - 2147483647 |  0      | How many objects to be placed in containers on workstations
tt_grasping            | 0 - 2147483647 |  0      | How many objects to be grapsed from RTTs and CBTs
tt_placing             | 0 - 2147483647 |  0      | How many objects to be placed on RTTs or CBTs
shelfes_grasping       | 0 - 2147483647 |  0      | How many objects to be grasped from Shelves
shelfes_placing        | 0 - 2147483647 |  0      | How many objects to be placed on Shelves
container_on_shelf     | 0 - 1          |  0      | If containers are allowed to be placed on shelfes
container_on_tt        | 0 - 1          |  0      | If containers are allowed ot be placed on RTTs and CBTs
ref_tt_direction       | 0 - 1          |  0      | If the direction of the RTT is set by the referees (Not implemented
yet)

#### String-List Configurations

Some advanced configurations need lists of strings as values:

Name                   |  Default                       | Description
-----------------------|--------------------------------|------------
cavities               |   {}                           | List of cavities to not be used for this task type
normalTableTypes       |   { "00", "05", "10", "15" }   | List of all workstation types, which are allowed to be used in this task type in addition to RTT, CBT, PPT and Shelf
allowedTables          |   {}                           | Explicit list of workstation names to be used in this task. Ignored if empty


#### String Configurations

Some special configuration parameters have string values:

Name                   |  Default | Description
-----------------------|----------|------------
ttTypes                |   "TT"   | Name of the workstation type indicating RTTs and CBTs
ppTypes                |   "PP"   | Name of the workstation type indicating PPTs
shTypes                |   "SH"   | Name of the workstation type indicating Shelfs

#### String-Integer Map

Special configuration option to declare the amount of objects to be used of each type of object for this task

Name                   |  Default | Description
-----------------------|----------|------------
objects                |   {}     | Explicit object count to declare the amount of objects of this type to be used (Not implemented in JurekGen)
